import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ActionNode(Node):
    def __init__(self):
        super().__init__("action_node")

        self.declare_parameter("cooldown_sec", 1.0)

        self.cooldown_sec = float(self.get_parameter("cooldown_sec").value)

        self.last_action_key = None
        self.last_action_time = 0.0

        self.action_sub = self.create_subscription(
            String,
            "/brain/actions",
            self.action_callback,
            10,
        )

        self.status_pub = self.create_publisher(
            String,
            "/action/status",
            10,
        )

        self.get_logger().info("Action node started. Listening on /brain/actions")

    def action_callback(self, msg):
        try:
            action = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Invalid action JSON: {msg.data}")
            return

        if not self.should_execute(action):
            return

        self.execute_action(action)

    def should_execute(self, action):
        """
        Prevents the same action from being executed too repeatedly.
        """
        now = time.time()

        action_type = action.get("action", "unknown")
        direction = action.get("direction", "")
        message = action.get("message", "")

        action_key = f"{action_type}:{direction}:{message}"

        if (
            action_key == self.last_action_key
            and now - self.last_action_time < self.cooldown_sec
        ):
            return False

        self.last_action_key = action_key
        self.last_action_time = now

        return True

    def execute_action(self, action):
        """
        For now, this does not move hardware.
        It only interprets and logs the action.

        Later, this is where motor control, LEDs, haptics, or navigation
        commands can be added.
        """
        action_type = action.get("action", "unknown")
        priority = action.get("priority", "normal")
        reason = action.get("reason", "")

        if action_type == "stop":
            self.handle_stop(action)

        elif action_type == "move_suggestion":
            self.handle_move_suggestion(action)

        elif action_type == "warn":
            self.handle_warning(action)

        elif action_type == "idle":
            self.handle_idle(action)

        else:
            self.get_logger().warn(f"Unknown action type: {action_type}")

        status = {
            "type": "action_status",
            "executed_action": action_type,
            "priority": priority,
            "reason": reason,
            "timestamp": time.time(),
        }

        status_msg = String()
        status_msg.data = json.dumps(status)
        self.status_pub.publish(status_msg)

    def handle_stop(self, action):
        reason = action.get("reason", "no reason provided")
        priority = action.get("priority", "high")

        self.get_logger().warn(
            f"[ACTION] STOP | priority={priority} | reason={reason}"
        )

        # Future motor command:
        # publish zero velocity
        # cmd_vel.linear.x = 0.0
        # cmd_vel.angular.z = 0.0

    def handle_move_suggestion(self, action):
        direction = action.get("direction", "unknown")
        reason = action.get("reason", "no reason provided")

        self.get_logger().info(
            f"[ACTION] MOVE SUGGESTION | direction={direction} | reason={reason}"
        )

        # Future motor command:
        # if direction == "left": turn left
        # if direction == "right": turn right
        # if direction == "forward": move forward

    def handle_warning(self, action):
        message = action.get("message", "warning")
        priority = action.get("priority", "normal")

        self.get_logger().warn(
            f"[ACTION] WARNING | priority={priority} | message={message}"
        )

        # Future hardware action:
        # flash LED
        # vibrate motor
        # play alert tone

    def handle_idle(self, action):
        self.get_logger().info("[ACTION] IDLE / NO ACTION")

        # Future:
        # maintain current state
        # keep motors stopped
        # low-power behavior


def main(args=None):
    rclpy.init(args=args)
    node = ActionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()