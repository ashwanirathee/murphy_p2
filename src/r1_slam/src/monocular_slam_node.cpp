#include "r1_slam/monocular_slam_node.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <utility>

namespace r1_slam
{

namespace
{

double rotationAngleFromMatrix(const cv::Mat & rotation)
{
  cv::Mat rotation_vector;
  cv::Rodrigues(rotation, rotation_vector);
  return cv::norm(rotation_vector);
}

}  // namespace

MonocularSlamNode::MonocularSlamNode()
: Node("monocular_slam_node"),
  publish_debug_image_(declare_parameter("publish_debug_image", true)),
  max_features_(declare_parameter("max_features", 600)),
  min_features_(declare_parameter("min_features", 150)),
  focal_length_(declare_parameter("focal_length", 320.0)),
  principal_point_x_(declare_parameter("principal_point_x", 160.0)),
  principal_point_y_(declare_parameter("principal_point_y", 120.0)),
  ransac_threshold_(declare_parameter("ransac_threshold", 1.0)),
  keyframe_translation_threshold_(declare_parameter("keyframe_translation_threshold", 0.05)),
  keyframe_rotation_threshold_rad_(declare_parameter("keyframe_rotation_threshold_rad", 0.03)),
  is_initialized_(false)
{
  camera_topic_ = declare_parameter("camera_topic", "/camera/uid_0/image_raw");
  world_frame_id_ = declare_parameter("world_frame_id", "slam_world");

  orb_detector_ = cv::ORB::create(max_features_);
  pose_rotation_ = cv::Mat::eye(3, 3, CV_64F);
  pose_translation_ = cv::Mat::zeros(3, 1, CV_64F);
  path_msg_.header.frame_id = world_frame_id_;

  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    camera_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&MonocularSlamNode::imageCallback, this, std::placeholders::_1));

  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/slam/pose", 10);
  path_pub_ = create_publisher<nav_msgs::msg::Path>("/slam/path", 10);
  debug_image_pub_ = create_publisher<sensor_msgs::msg::Image>("/slam/debug_image", 10);

  RCLCPP_INFO(get_logger(), "Monocular SLAM node listening on %s", camera_topic_.c_str());
}

void MonocularSlamNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  cv::Mat frame;

  try {
    frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
  } catch (const cv_bridge::Exception & exc) {
    RCLCPP_WARN(get_logger(), "cv_bridge failed: %s", exc.what());
    return;
  }

  if (frame.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Received empty image frame");
    return;
  }

  cv::Mat gray_frame;
  cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);

  if (!is_initialized_) {
    if (initializeFromFrame(gray_frame)) {
      last_frame_time_ = msg->header.stamp;
      publishPose(msg->header);
    }
    return;
  }

  processTrackedFrame(gray_frame, msg->header);
}

bool MonocularSlamNode::initializeFromFrame(const cv::Mat & gray_frame)
{
  std::vector<cv::KeyPoint> keypoints;
  orb_detector_->detect(gray_frame, keypoints);

  if (keypoints.size() < static_cast<size_t>(min_features_)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Waiting for more texture to initialize SLAM: %zu features", keypoints.size());
    return false;
  }

  previous_points_.clear();
  previous_points_.reserve(keypoints.size());

  for (const auto & keypoint : keypoints) {
    previous_points_.push_back(keypoint.pt);
  }

  previous_gray_ = gray_frame.clone();
  is_initialized_ = true;

  RCLCPP_INFO(get_logger(), "Initialized monocular tracker with %zu features", previous_points_.size());
  return true;
}

void MonocularSlamNode::processTrackedFrame(
  const cv::Mat & gray_frame,
  const std_msgs::msg::Header & header)
{
  if (previous_points_.size() < static_cast<size_t>(min_features_)) {
    detectNewFeatures(previous_gray_);
  }

  if (previous_points_.size() < 8U) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Too few points to estimate pose");
    previous_gray_ = gray_frame.clone();
    detectNewFeatures(previous_gray_);
    return;
  }

  std::vector<cv::Point2f> tracked_points;
  std::vector<unsigned char> track_status;
  std::vector<float> tracking_error;

  cv::calcOpticalFlowPyrLK(
    previous_gray_,
    gray_frame,
    previous_points_,
    tracked_points,
    track_status,
    tracking_error);

  std::vector<cv::Point2f> filtered_previous_points;
  std::vector<cv::Point2f> filtered_current_points;

  for (size_t i = 0; i < track_status.size(); ++i) {
    if (!track_status[i]) {
      continue;
    }

    const auto & previous_point = previous_points_[i];
    const auto & current_point = tracked_points[i];

    if (
      current_point.x < 0.0F || current_point.y < 0.0F ||
      current_point.x >= static_cast<float>(gray_frame.cols) ||
      current_point.y >= static_cast<float>(gray_frame.rows))
    {
      continue;
    }

    filtered_previous_points.push_back(previous_point);
    filtered_current_points.push_back(current_point);
  }

  if (filtered_current_points.size() < 8U) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Tracking degraded: only %zu inlier candidates", filtered_current_points.size());
    previous_gray_ = gray_frame.clone();
    detectNewFeatures(previous_gray_);
    return;
  }

  const cv::Point2d principal_point(principal_point_x_, principal_point_y_);
  cv::Mat essential_inlier_mask;
  const cv::Mat essential_matrix = cv::findEssentialMat(
    filtered_current_points,
    filtered_previous_points,
    focal_length_,
    principal_point,
    cv::RANSAC,
    0.999,
    ransac_threshold_,
    essential_inlier_mask);

  if (essential_matrix.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Essential matrix estimation failed");
    previous_gray_ = gray_frame.clone();
    detectNewFeatures(previous_gray_);
    return;
  }

  cv::Mat relative_rotation;
  cv::Mat relative_translation;
  const int recovered_points = cv::recoverPose(
    essential_matrix,
    filtered_current_points,
    filtered_previous_points,
    relative_rotation,
    relative_translation,
    focal_length_,
    principal_point,
    essential_inlier_mask);

  if (recovered_points < 8) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Pose recovery produced too few inliers: %d", recovered_points);
    previous_gray_ = gray_frame.clone();
    detectNewFeatures(previous_gray_);
    return;
  }

  const double translation_step = cv::norm(relative_translation);
  const double rotation_step = rotationAngleFromMatrix(relative_rotation);

  if (
    translation_step >= keyframe_translation_threshold_ ||
    rotation_step >= keyframe_rotation_threshold_rad_)
  {
    pose_translation_ += pose_rotation_ * relative_translation;
    pose_rotation_ = relative_rotation * pose_rotation_;
  }

  previous_gray_ = gray_frame.clone();
  previous_points_ = filtered_current_points;

  if (static_cast<int>(previous_points_.size()) < min_features_) {
    detectNewFeatures(previous_gray_);
  }

  publishPose(header);

  if (publish_debug_image_) {
    publishDebugImage(gray_frame, filtered_previous_points, filtered_current_points);
  }
}

void MonocularSlamNode::detectNewFeatures(const cv::Mat & gray_frame)
{
  std::vector<cv::KeyPoint> keypoints;
  orb_detector_->detect(gray_frame, keypoints);

  if (keypoints.empty()) {
    return;
  }

  std::vector<cv::Point2f> refreshed_points;
  refreshed_points.reserve(std::min(static_cast<int>(keypoints.size()), max_features_));

  for (const auto & keypoint : keypoints) {
    refreshed_points.push_back(keypoint.pt);
    if (static_cast<int>(refreshed_points.size()) >= max_features_) {
      break;
    }
  }

  previous_points_ = std::move(refreshed_points);
}

void MonocularSlamNode::publishPose(const std_msgs::msg::Header & header)
{
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header = header;
  pose_msg.header.frame_id = world_frame_id_;
  pose_msg.pose.position.x = pose_translation_.at<double>(0, 0);
  pose_msg.pose.position.y = pose_translation_.at<double>(1, 0);
  pose_msg.pose.position.z = pose_translation_.at<double>(2, 0);
  pose_msg.pose.orientation = rotationMatrixToQuaternion(pose_rotation_);

  pose_pub_->publish(pose_msg);

  path_msg_.header.stamp = header.stamp;
  path_msg_.poses.push_back(pose_msg);
  path_pub_->publish(path_msg_);
}

void MonocularSlamNode::publishDebugImage(
  const cv::Mat & gray_frame,
  const std::vector<cv::Point2f> & previous_points,
  const std::vector<cv::Point2f> & current_points)
{
  cv::Mat debug_bgr;
  cv::cvtColor(gray_frame, debug_bgr, cv::COLOR_GRAY2BGR);

  const size_t count = std::min(previous_points.size(), current_points.size());
  for (size_t i = 0; i < count; ++i) {
    cv::line(debug_bgr, previous_points[i], current_points[i], cv::Scalar(0, 255, 0), 1);
    cv::circle(debug_bgr, current_points[i], 2, cv::Scalar(0, 0, 255), -1);
  }

  auto debug_msg = cv_bridge::CvImage(
    std_msgs::msg::Header(),
    "bgr8",
    debug_bgr).toImageMsg();
  debug_msg->header.frame_id = world_frame_id_;
  debug_msg->header.stamp = now();
  debug_image_pub_->publish(*debug_msg);
}

geometry_msgs::msg::Quaternion MonocularSlamNode::rotationMatrixToQuaternion(
  const cv::Mat & rotation)
{
  geometry_msgs::msg::Quaternion quaternion;

  const double trace = rotation.at<double>(0, 0) + rotation.at<double>(1, 1) + rotation.at<double>(2, 2);

  if (trace > 0.0) {
    const double scale = std::sqrt(trace + 1.0) * 2.0;
    quaternion.w = 0.25 * scale;
    quaternion.x = (rotation.at<double>(2, 1) - rotation.at<double>(1, 2)) / scale;
    quaternion.y = (rotation.at<double>(0, 2) - rotation.at<double>(2, 0)) / scale;
    quaternion.z = (rotation.at<double>(1, 0) - rotation.at<double>(0, 1)) / scale;
  } else if (rotation.at<double>(0, 0) > rotation.at<double>(1, 1) &&
    rotation.at<double>(0, 0) > rotation.at<double>(2, 2))
  {
    const double scale = std::sqrt(1.0 + rotation.at<double>(0, 0) -
      rotation.at<double>(1, 1) - rotation.at<double>(2, 2)) * 2.0;
    quaternion.w = (rotation.at<double>(2, 1) - rotation.at<double>(1, 2)) / scale;
    quaternion.x = 0.25 * scale;
    quaternion.y = (rotation.at<double>(0, 1) + rotation.at<double>(1, 0)) / scale;
    quaternion.z = (rotation.at<double>(0, 2) + rotation.at<double>(2, 0)) / scale;
  } else if (rotation.at<double>(1, 1) > rotation.at<double>(2, 2)) {
    const double scale = std::sqrt(1.0 + rotation.at<double>(1, 1) -
      rotation.at<double>(0, 0) - rotation.at<double>(2, 2)) * 2.0;
    quaternion.w = (rotation.at<double>(0, 2) - rotation.at<double>(2, 0)) / scale;
    quaternion.x = (rotation.at<double>(0, 1) + rotation.at<double>(1, 0)) / scale;
    quaternion.y = 0.25 * scale;
    quaternion.z = (rotation.at<double>(1, 2) + rotation.at<double>(2, 1)) / scale;
  } else {
    const double scale = std::sqrt(1.0 + rotation.at<double>(2, 2) -
      rotation.at<double>(0, 0) - rotation.at<double>(1, 1)) * 2.0;
    quaternion.w = (rotation.at<double>(1, 0) - rotation.at<double>(0, 1)) / scale;
    quaternion.x = (rotation.at<double>(0, 2) + rotation.at<double>(2, 0)) / scale;
    quaternion.y = (rotation.at<double>(1, 2) + rotation.at<double>(2, 1)) / scale;
    quaternion.z = 0.25 * scale;
  }

  return quaternion;
}

}  // namespace r1_slam

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<r1_slam::MonocularSlamNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
