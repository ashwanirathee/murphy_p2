import subprocess
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class AudioNode(Node):
    def __init__(self):
        super().__init__("audio_node")

        self.declare_parameter("cooldown_sec", 2.0)
        self.declare_parameter("use_espeak", True)

        self.cooldown_sec = float(self.get_parameter("cooldown_sec").value)
        self.use_espeak = bool(self.get_parameter("use_espeak").value)

        self.last_text = None
        self.last_time = 0.0

        self.sub = self.create_subscription(
            String,
            "/audio/speech_text",
            self.speech_callback,
            10,
        )

        self.get_logger().info("Audio node started. Listening on /audio/speech_text")

    def speech_callback(self, msg):
        text = msg.data.strip()

        if not text:
            return

        now = time.time()

        # Avoid repeating same message too often
        if text == self.last_text and now - self.last_time < self.cooldown_sec:
            return

        self.last_text = text
        self.last_time = now

        self.get_logger().info(f"Speaking: {text}")

        if self.use_espeak:
            self.speak(text)
        else:
            print(f"[AUDIO] {text}")

    def speak(self, text):
        try:
            subprocess.Popen(
                ["espeak-ng", "-v", "en-us", "-s", "130", "-p", "35", "-a", "120", "-g", "8", text],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except FileNotFoundError:
            self.get_logger().warn("espeak-ng not found. Printing instead.")
            print(f"[AUDIO] {text}")


def main(args=None):
    rclpy.init(args=args)
    node = AudioNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()