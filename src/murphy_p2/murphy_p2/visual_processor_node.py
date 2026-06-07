import json
import cv2
import numpy as np
import rclpy
import os
import atexit
import shutil
import signal
import time
from datetime import datetime
from pathlib import Path

from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


class VisualProcessorNode(Node):
    def __init__(self):
        super().__init__("visual_processor_node")

        self.bridge = CvBridge()
        self._cleanup_done = False

        # Image saving setup with memory management
        self.image_save_dir = os.path.expanduser("~/murphy_p2/vision_images")
        os.makedirs(self.image_save_dir, exist_ok=True)
        
        self.workspace_dir = os.path.expanduser("~/murphy_p2")
        self.latest_image_path = os.path.join(self.workspace_dir, "latest_frame.jpg")
        self.latest_image_meta_path = os.path.join(self.workspace_dir, "latest_frame.json")
        
        # Memory/disk management parameters
        self.max_images = 10  # Keep only 100 most recent images
        self.max_dir_size_mb = 50  # Max directory size in MB

        self.declare_parameter("camera_uids", [0, 1])
        self.declare_parameter("camera_labels", ["left", "right"])
        self.declare_parameter("event_min_interval_sec", 0.5)
        self.declare_parameter("event_max_silence_sec", 2.0)

        camera_uids = list(self.get_parameter("camera_uids").value)
        camera_labels = list(self.get_parameter("camera_labels").value)
        self.event_min_interval_sec = float(
            self.get_parameter("event_min_interval_sec").value
        )
        self.event_max_silence_sec = float(
            self.get_parameter("event_max_silence_sec").value
        )

        if len(camera_labels) < len(camera_uids):
            camera_labels.extend(str(uid) for uid in camera_uids[len(camera_labels) :])

        self.camera_states = {}
        for uid, label in zip(camera_uids, camera_labels):
            self.camera_states[uid] = {
                "label": label,
                "latest_frame": None,
                "last_obstacle_state": None,
            }

            self.create_subscription(
                Image,
                f"/camera/uid_{uid}/image_raw",
                lambda msg, camera_uid=uid: self.image_callback(msg, camera_uid),
                10,
            )

        # Timer for periodic image saving (every 10 seconds)
        self.create_timer(10.0, self.save_image_periodic)

        self.event_pub = self.create_publisher(String, "/vision/events", 10)
        self.last_published_event_keys = {}
        self.last_event_publish_times = {}

        atexit.register(self._cleanup_image_directory)
        signal.signal(signal.SIGINT, self._handle_shutdown_signal)
        signal.signal(signal.SIGTERM, self._handle_shutdown_signal)

        self.get_logger().info("Visual processor node started.")

    def image_callback(self, msg, camera_uid):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.save_latest_frame(frame, camera_uid)
        camera_state = self.camera_states.get(camera_uid)
        if camera_state is None:
            self.get_logger().warning(f"Received frame for unknown camera uid {camera_uid}.")
            return

        camera_state["latest_frame"] = frame

        event = self.analyze_frame(frame)
        event["camera_uid"] = camera_uid
        event["camera_label"] = camera_state["label"]

        # Save image when decision changes (obstacle detected or cleared)
        if camera_state["last_obstacle_state"] != event["obstacle_like"]:
            self.save_image_decision(frame, event)
            camera_state["last_obstacle_state"] = event["obstacle_like"]

        if self.should_publish_event(event):
            out_msg = String()
            out_msg.data = json.dumps(event)
            self.event_pub.publish(out_msg)

    def save_latest_frame(self, frame, camera_uid):
        """
        Save a stable latest image for the VLM node.
        This file is overwritten every frame, so it does not grow disk usage.
        """
        cv2.imwrite(self.latest_image_path, frame)

        camera_state = self.camera_states.get(camera_uid, {})
        meta = {
            "image_path": self.latest_image_path,
            "camera_uid": camera_uid,
            "camera_label": camera_state.get("label", "unknown"),
            "timestamp": datetime.now().isoformat(),
        }

        with open(self.latest_image_meta_path, "w") as f:
            json.dump(meta, f)

    def save_image_periodic(self):
        """Save the latest frame every 10 seconds"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        for camera_uid, camera_state in self.camera_states.items():
            frame = camera_state["latest_frame"]
            if frame is None:
                continue

            label = camera_state["label"]
            filename = os.path.join(
                self.image_save_dir,
                f"frame_periodic_{camera_uid}_{label}_{timestamp}.jpg",
            )
            cv2.imwrite(filename, frame)
            self.get_logger().info(f"Saved periodic image: {filename}")

        self.cleanup_old_images()

    def save_image_decision(self, frame, event):
        """Save image when a decision is made (obstacle state changes)"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        decision = "obstacle" if event["obstacle_like"] else "clear"
        camera_uid = event.get("camera_uid", "unknown")
        camera_label = event.get("camera_label", "unknown")
        filename = os.path.join(
            self.image_save_dir,
            f"frame_decision_{camera_uid}_{camera_label}_{decision}_{timestamp}.jpg",
        )
        cv2.imwrite(filename, frame)
        self.get_logger().info(
            f"Saved decision image from uid {camera_uid} ({camera_label}): {filename}"
        )
        self.cleanup_old_images()

    def cleanup_old_images(self):
        """Remove old images to prevent unbounded disk growth"""
        try:
            files = sorted(
                Path(self.image_save_dir).glob("*.jpg"),
                key=lambda p: p.stat().st_mtime,
                reverse=True,
            )

            # Check max image count
            if len(files) > self.max_images:
                for old_file in files[self.max_images:]:
                    old_file.unlink()
                    self.get_logger().debug(f"Deleted old image: {old_file.name}")

            # Check total directory size
            total_size_mb = sum(f.stat().st_size for f in files) / (1024 * 1024)
            if total_size_mb > self.max_dir_size_mb:
                for old_file in files[self.max_images // 2:]:
                    old_file.unlink()
                    self.get_logger().debug(
                        f"Deleted image due to size limit: {old_file.name}"
                    )
        except Exception as e:
            self.get_logger().warning(f"Error during image cleanup: {e}")

    def should_publish_event(self, event):
        camera_uid = event.get("camera_uid", "unknown")
        now = time.time()
        event_key = (
            bool(event.get("obstacle_like", False)),
            event.get("region", "unknown"),
        )

        last_key = self.last_published_event_keys.get(camera_uid)
        last_time = self.last_event_publish_times.get(camera_uid, 0.0)
        elapsed = now - last_time

        if last_key is None:
            should_publish = True
        elif event_key != last_key and elapsed >= self.event_min_interval_sec:
            should_publish = True
        elif elapsed >= self.event_max_silence_sec:
            should_publish = True
        else:
            should_publish = False

        if should_publish:
            self.last_published_event_keys[camera_uid] = event_key
            self.last_event_publish_times[camera_uid] = now

        return should_publish

    def destroy_node(self):
        self._cleanup_image_directory()
        super().destroy_node()

    def _handle_shutdown_signal(self, signum, frame):
        self._cleanup_image_directory()
        rclpy.shutdown()

    def _cleanup_image_directory(self):
        if self._cleanup_done:
            return

        self._cleanup_done = True

        try:
            if os.path.isdir(self.image_save_dir):
                shutil.rmtree(self.image_save_dir)
                self.get_logger().info(f"Deleted image save directory: {self.image_save_dir}")
        except Exception as e:
            self.get_logger().warning(f"Failed to delete image save directory: {e}")

    def analyze_frame(self, frame):
        height, width, _ = frame.shape

        # Resize for cheaper processing on Raspberry Pi
        small = cv2.resize(frame, (320, 240))

        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 80, 160)

        h, w = gray.shape

        # Divide image into left, center, right regions
        left_region = edges[:, : w // 3]
        center_region = edges[:, w // 3 : 2 * w // 3]
        right_region = edges[:, 2 * w // 3 :]

        left_score = int(np.sum(left_region > 0))
        center_score = int(np.sum(center_region > 0))
        right_score = int(np.sum(right_region > 0))

        scores = {
            "left": left_score,
            "center": center_score,
            "right": right_score,
        }

        strongest_region = max(scores, key=scores.get)
        strongest_score = scores[strongest_region]

        # Simple threshold. Tune this on your camera feed.
        obstacle_like = strongest_score > 2500

        event = {
            "type": "visual_observation",
            "obstacle_like": obstacle_like,
            "region": strongest_region,
            "scores": scores,
            "image_width": int(width),
            "image_height": int(height),
        }

        return event


def main(args=None):
    rclpy.init(args=args)
    node = VisualProcessorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
