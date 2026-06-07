import cv2
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CamerasNode(Node):
    def __init__(self):
        super().__init__("cameras_node")

        self.declare_parameter("camera_uids", [0, 1])
        self.declare_parameter("camera_labels", ["left", "right"])
        self.declare_parameter("fps", 15)

        camera_uids = list(self.get_parameter("camera_uids").value)
        camera_labels = list(self.get_parameter("camera_labels").value)
        fps = self.get_parameter("fps").value

        if len(camera_labels) < len(camera_uids):
            camera_labels.extend(str(uid) for uid in camera_uids[len(camera_labels) :])

        self.cameras = []
        for uid, label in zip(camera_uids, camera_labels):
            capture = cv2.VideoCapture(uid)

            if not capture.isOpened():
                self.get_logger().error(f"Could not open camera uid {uid}")

            topic_name = f"/camera/uid_{uid}/image_raw"
            publisher = self.create_publisher(Image, topic_name, 10)
            self.cameras.append(
                {
                    "uid": uid,
                    "label": label,
                    "capture": capture,
                    "publisher": publisher,
                    "topic_name": topic_name,
                }
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

            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = timestamp
            msg.header.frame_id = f"camera_uid_{camera['uid']}"

            camera["publisher"].publish(msg)

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