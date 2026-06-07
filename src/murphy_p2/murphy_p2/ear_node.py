import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class EarNode(Node):
    def __init__(self):
        super().__init__("ear_node")

        self.pub = self.create_publisher(String, "/audio/heard_text", 10)

        self.get_logger().info("Ear node started.")
        self.get_logger().info("Type text and press Enter. It will publish to /audio/heard_text")

        self.input_thread = threading.Thread(
            target=self.read_terminal_input,
            daemon=True,
        )
        self.input_thread.start()

    def read_terminal_input(self):
        while rclpy.ok():
            try:
                text = input("> ").strip()
            except EOFError:
                break

            if not text:
                continue

            msg = String()
            msg.data = text
            self.pub.publish(msg)

            self.get_logger().info(f"Heard: {text}")


def main(args=None):
    rclpy.init(args=args)
    node = EarNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()