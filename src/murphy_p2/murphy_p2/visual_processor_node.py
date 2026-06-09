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
from sensor_msgs.msg import CompressedImage, Image
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
        self.declare_parameter("enable_2dobd", True)
        self.declare_parameter("enable_3dobd", False)
        self.declare_parameter("yolo_camera_uid", 0)
        self.declare_parameter("yolo_debug_jpeg_quality", 70)

        camera_uids = list(self.get_parameter("camera_uids").value)
        camera_labels = list(self.get_parameter("camera_labels").value)
        self.event_min_interval_sec = float(
            self.get_parameter("event_min_interval_sec").value
        )
        self.event_max_silence_sec = float(
            self.get_parameter("event_max_silence_sec").value
        )
        self.enable_2dobd = bool(self.get_parameter("enable_2dobd").value)
        self.enable_3dobd = bool(self.get_parameter("enable_3dobd").value)
        self.yolo_camera_uid = int(self.get_parameter("yolo_camera_uid").value)
        self.yolo_debug_jpeg_quality = int(
            self.get_parameter("yolo_debug_jpeg_quality").value
        )

        if len(camera_labels) < len(camera_uids):
            camera_labels.extend(str(uid) for uid in camera_uids[len(camera_labels) :])

        if self.yolo_camera_uid not in camera_uids:
            self.get_logger().warning(
                f"yolo_camera_uid {self.yolo_camera_uid} is not in camera_uids {camera_uids}. "
                "YOLO detections will never run until they match."
            )

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
        # self.create_timer(10.0, self.save_image_periodic)

        self.event_pub = self.create_publisher(String, "/vision/events", 10)
        self.last_published_event_keys = {}
        self.last_event_publish_times = {}

        atexit.register(self._cleanup_image_directory)
        signal.signal(signal.SIGINT, self._handle_shutdown_signal)
        signal.signal(signal.SIGTERM, self._handle_shutdown_signal)

        self.get_logger().info("Visual processor node started.")
        self.get_logger().info(
            f"OBD modes: 2D={self.enable_2dobd}, 3D={self.enable_3dobd}"
        )
        self.get_logger().info(f"YOLO camera uid: {self.yolo_camera_uid}")

        if self.enable_2dobd:
            from ultralytics import YOLO

            self.model = None
            self.yolo_imgsz = 320
            self.yolo_conf = 0.4
            self.yolo_min_interval_sec = 0.5  # 2 FPS max
            self.last_yolo_time = 0.0
            self.last_yolo_event = None
            self.yolo_debug_pub = None

            self.model = YOLO("~/murphy_p2/yolov8n.pt")
            self.yolo_debug_pub = self.create_publisher(
                CompressedImage,
                "/vision/yolo_debug_image/compressed",
                1,
            )

    def image_callback(self, msg, camera_uid):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        camera_state = self.camera_states.get(camera_uid)
        if camera_state is None:
            self.get_logger().warning(f"Received frame for unknown camera uid {camera_uid}.")
            return

        camera_state["latest_frame"] = frame

        now = time.time()

        if self.enable_2dobd and camera_uid == self.yolo_camera_uid:
            # Run YOLO at limited rate
            if now - self.last_yolo_time >= self.yolo_min_interval_sec:
                self.last_yolo_time = now
                event = self.analyze_frame_yolo(frame)
                self.last_yolo_event = event
            elif self.last_yolo_event is not None:
                event = dict(self.last_yolo_event)
            else:
                event = self.analyze_frame(frame)
        else:
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

    def analyze_frame_yolo(self, frame):
        height, width, _ = frame.shape

        results = self.model.predict(
            frame,
            imgsz=self.yolo_imgsz,
            device="cpu",
            conf=self.yolo_conf,
            verbose=False,
        )

        debug_frame = frame.copy()

        detections = []
        obstacle_like = False
        strongest_region = "unknown"
        best_conf = 0.0

        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                label = self.model.names[cls_id]

                x1, y1, x2, y2 = box.xyxy[0].tolist()
                x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])

                cx = (x1 + x2) / 2.0
                if cx < width / 3:
                    region = "left"
                elif cx < 2 * width / 3:
                    region = "center"
                else:
                    region = "right"

                # Draw YOLO box
                cv2.rectangle(debug_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                text = f"{label} {conf:.2f} {region}"
                cv2.putText(
                    debug_frame,
                    text,
                    (x1, max(20, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                )

                detections.append({
                    "label": label,
                    "confidence": conf,
                    "bbox": [x1, y1, x2, y2],
                    "region": region,
                })

                if label in ["person", "chair", "couch", "bed", "dining table", "tv", "backpack"]:
                    obstacle_like = True
                    if conf > best_conf:
                        best_conf = conf
                        strongest_region = region

        # Optional: draw region dividers
        cv2.line(debug_frame, (width // 3, 0), (width // 3, height), (255, 255, 0), 1)
        cv2.line(debug_frame, (2 * width // 3, 0), (2 * width // 3, height), (255, 255, 0), 1)

        # Publish compressed debug image to reduce transport bandwidth.
        if self.yolo_debug_pub is not None:
            ok, encoded = cv2.imencode(
                ".jpg",
                debug_frame,
                [int(cv2.IMWRITE_JPEG_QUALITY), self.yolo_debug_jpeg_quality],
            )
            if ok:
                debug_msg = CompressedImage()
                debug_msg.header.stamp = self.get_clock().now().to_msg()
                debug_msg.header.frame_id = "yolo_debug"
                debug_msg.format = "jpeg"
                debug_msg.data = encoded.tobytes()
                self.yolo_debug_pub.publish(debug_msg)

        if not obstacle_like:
            strongest_region = "unknown"

        return {
            "type": "yolo_observation",
            "obstacle_like": obstacle_like,
            "region": strongest_region,
            "detections": detections,
            "image_width": int(width),
            "image_height": int(height),
        }

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
