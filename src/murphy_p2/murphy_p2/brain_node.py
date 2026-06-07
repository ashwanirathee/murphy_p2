import json
import rclpy

from rclpy.node import Node
from std_msgs.msg import String


class BrainNode(Node):
    def __init__(self):
        super().__init__("brain_node")

        self.vision_sub = self.create_subscription(
            String,
            "/vision/events",
            self.vision_callback,
            10,
        )

        self.heard_sub = self.create_subscription(
            String,
            "/audio/heard_text",
            self.heard_callback,
            10,
        )

        self.speech_pub = self.create_publisher(
            String, 
            "/audio/speech_text", 
            10
        )

        self.last_vision_decision = None
        self.last_message = None

        self.get_logger().info("Brain node started.")

    def vision_callback(self, msg):
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("Invalid JSON from visual processor.")
            return

        self.last_event = event

        decision = self.make_vision_decision(event)

        if decision is None:
            return

        if decision == self.last_vision_decision:
            return

        self.last_vision_decision = decision
        self.say(decision)

    def make_vision_decision(self, event):
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

    def heard_callback(self, msg):
        text = msg.data.strip().lower()

        self.get_logger().info(f"Received user command: {text}")

        if text in ["hello", "hi"]:
            self.say("Hello. Murphy P2 is online.")

        elif text in ["stop", "quiet", "silence"]:
            self.say("Okay. I will stay quiet.")

        elif text in ["what do you see", "describe", "describe scene"]:
            self.describe_last_event()

        elif text in ["status", "system status"]:
            self.say("Camera, visual processor, brain, and audio nodes are active.")

        else:
            self.say(f"I heard: {text}")

    def describe_last_event(self):
        if self.last_event is None:
            self.say("I do not have a visual observation yet.")
            return

        obstacle_like = self.last_event.get("obstacle_like", False)
        region = self.last_event.get("region", "unknown")

        if obstacle_like:
            self.say(f"I see an obstacle like region on the {region}.")
        else:
            self.say("I do not currently see a strong obstacle like region.")

    def say(self, text):
        msg = String()
        msg.data = text
        self.speech_pub.publish(msg)
        self.get_logger().info(f"Brain says: {text}")

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