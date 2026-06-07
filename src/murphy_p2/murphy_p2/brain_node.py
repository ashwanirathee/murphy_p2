import json
import rclpy

from rclpy.node import Node
from std_msgs.msg import String


class BrainNode(Node):
    def __init__(self):
        super().__init__("brain_node")

        self.sub = self.create_subscription(
            String,
            "/vision/events",
            self.event_callback,
            10,
        )

        # self.speech_pub = self.create_publisher(String, "/audio/speech_text", 10)

        self.last_message = None

        self.get_logger().info("Brain node started.")

    def event_callback(self, msg):
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("Received invalid JSON from visual processor.")
            return

        decision = self.make_decision(event)

        if decision is None:
            return

        # Avoid repeating same message constantly
        if decision == self.last_message:
            return

        self.last_message = decision

        out_msg = String()
        out_msg.data = decision
        # self.speech_pub.publish(out_msg)

        self.get_logger().info(f"Decision: {decision}")

    def make_decision(self, event):
        obstacle_like = event.get("obstacle_like", False)
        region = event.get("region", "center")

        if not obstacle_like:
            return None

        if region == "center":
            return "Obstacle ahead."
        elif region == "left":
            return "Obstacle on the left."
        elif region == "right":
            return "Obstacle on the right."

        return "Obstacle detected."


def main(args=None):
    rclpy.init(args=args)
    node = BrainNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()