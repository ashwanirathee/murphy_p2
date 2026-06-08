import cv2
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

class CamerasNode(Node):
    def __init__(self):
        super().__init__("cameras_node")

        self.declare_parameter("camera_uids", [0, 1])
        self.declare_parameter("camera_labels", ["left", "right"])
        self.declare_parameter("fps", 30)
        self.declare_parameter("jpeg_quality", 70)

        camera_uids = list(self.get_parameter("camera_uids").value)
        camera_labels = list(self.get_parameter("camera_labels").value)
        fps = self.get_parameter("fps").value
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)

        if len(camera_labels) < len(camera_uids):
            camera_labels.extend(str(uid) for uid in camera_uids[len(camera_labels) :])

        self.cameras = []
        for uid, label in zip(camera_uids, camera_labels):
            capture = cv2.VideoCapture(uid)

            # Ask USB camera/OpenCV for MJPEG capture if supported.
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            capture.set(cv2.CAP_PROP_FPS, fps)
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

            if not capture.isOpened():
                self.get_logger().error(f"Could not open camera uid {uid}")

            raw_topic_name = f"/camera/uid_{uid}/image_raw"
            compressed_topic_name = f"/camera/uid_{uid}/image_compressed"

            raw_publisher = self.create_publisher(Image, raw_topic_name, 10)
            compressed_publisher = self.create_publisher(CompressedImage, compressed_topic_name, 10)
            
            self.cameras.append(
                {
                    "uid": uid,
                    "label": label,
                    "capture": capture,
                    "raw_publisher": raw_publisher,
                    "compressed_publisher": compressed_publisher,
                    "raw_topic_name": raw_topic_name,
                    "compressed_topic_name": compressed_topic_name,
                }
            )

            self.get_logger().info(
                f"Camera uid {uid} ({label}) publishing raw={raw_topic_name}, "
                f"compressed={compressed_topic_name}"
            )

        self.bridge = CvBridge()

        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.publish_frames)

        self.get_logger().info("Cameras node started.")

    def publish_frames(self):
        timestamp = self.get_clock().now().to_msg()

        for camera in self.cameras:
            ret, frame = camera["capture"].read()

            if not ret:
                self.get_logger().warn(f"Failed to read camera uid {camera['uid']}.")
                continue

            raw_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            raw_msg.header.stamp = timestamp
            raw_msg.header.frame_id = f"camera_uid_{camera['uid']}"

            camera["raw_publisher"].publish(raw_msg)

            ok, encoded = cv2.imencode(
                ".jpg",
                frame,
                [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
            )

            if ok:
                compressed_msg = CompressedImage()
                compressed_msg.header.stamp = timestamp
                compressed_msg.header.frame_id = f"camera_uid_{camera['uid']}"
                compressed_msg.format = "jpeg"
                compressed_msg.data = encoded.tobytes()
                camera["compressed_publisher"].publish(compressed_msg)
    def destroy_node(self):
        for camera in self.cameras:
            camera["capture"].release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CamerasNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()