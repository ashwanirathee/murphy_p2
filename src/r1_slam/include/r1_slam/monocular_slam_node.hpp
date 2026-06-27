#ifndef R1_SLAM__MONOCULAR_SLAM_NODE_HPP_
#define R1_SLAM__MONOCULAR_SLAM_NODE_HPP_

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <memory>
#include <string>
#include <vector>

namespace r1_slam
{

class MonocularSlamNode : public rclcpp::Node
{
public:
  MonocularSlamNode();

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  bool initializeFromFrame(const cv::Mat & gray_frame);
  void processTrackedFrame(const cv::Mat & gray_frame, const std_msgs::msg::Header & header);
  void detectNewFeatures(const cv::Mat & gray_frame);
  void publishPose(const std_msgs::msg::Header & header);
  void publishDebugImage(
    const cv::Mat & gray_frame,
    const std::vector<cv::Point2f> & previous_points,
    const std::vector<cv::Point2f> & current_points);
  static geometry_msgs::msg::Quaternion rotationMatrixToQuaternion(const cv::Mat & rotation);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;

  std::string camera_topic_;
  std::string world_frame_id_;
  bool publish_debug_image_;
  int max_features_;
  int min_features_;
  double focal_length_;
  double principal_point_x_;
  double principal_point_y_;
  double ransac_threshold_;
  double keyframe_translation_threshold_;
  double keyframe_rotation_threshold_rad_;

  cv::Mat previous_gray_;
  std::vector<cv::Point2f> previous_points_;
  cv::Mat pose_rotation_;
  cv::Mat pose_translation_;
  nav_msgs::msg::Path path_msg_;
  rclcpp::Time last_frame_time_;
  bool is_initialized_;

  cv::Ptr<cv::ORB> orb_detector_;
};

}  // namespace r1_slam

#endif  // R1_SLAM__MONOCULAR_SLAM_NODE_HPP_
