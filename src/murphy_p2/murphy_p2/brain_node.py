import json
import rclpy
import os

from rclpy.node import Node
from std_msgs.msg import String

import requests
import time

import cv2
import base64
import threading
import numpy as np
from sensor_msgs.msg import CompressedImage

class BrainNode(Node):
    def __init__(self):
        super().__init__("brain_node")
        self.declare_parameter(
            "latest_image_path",
            "/home/ubuntu/murphy_p2/latest_frame.jpg",
        )

        self.latest_image_path = self.get_parameter("latest_image_path").value

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
            10,
        )

        # New: publish structured robot actions
        self.action_pub = self.create_publisher(
            String,
            "/brain/actions",
            10,
        )

        self.vlm_question_pub = self.create_publisher(
            String,
            "/vlm/questions",
            10,
        )

        self.vlm_answer_sub = self.create_subscription(
            String,
            "/vlm/answers",
            self.vlm_answer_callback,
            10,
        )

        self.last_vision_decision = None
        self.last_action_key = None
        self.last_event = None
        self.last_vlm_question = None

        self.quiet_mode = False

        self.latest_yolo_jpeg_bytes = None
        self.latest_yolo_image_time = 0.0

        self.yolo_image_sub = self.create_subscription(
            CompressedImage,
            "/vision/yolo_debug_image/compressed",
            self.yolo_image_callback,
            10,
        )

        self.ollama_vision_url = "http://192.168.68.51:8000/image-and-say"

        self.last_face_speak_time = 0.0
        self.face_speak_cooldown_sec = 8.0
        self.last_face_region = None
        self.face_request_in_flight = False
        
        self.get_logger().info("Brain node started.")

    def yolo_image_callback(self, msg):
        self.latest_yolo_jpeg_bytes = bytes(msg.data)
        self.latest_yolo_image_time = time.time()

    def vision_callback(self, msg):
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("Invalid JSON from visual processor.")
            return

        self.last_event = event
        if event.get("type") == "face_detection_observation":
            self.maybe_say_face_event(event)
            return 

        decision, action = self.make_vision_decision(event)

        if action is not None:
            self.publish_action(action)

        if decision is None:
            return

        if decision == self.last_vision_decision:
            return

        self.last_vision_decision = decision

        if not self.quiet_mode:
            self.say(decision)

    def make_vision_decision(self, event):
        obstacle_like = event.get("obstacle_like", False)
        region = event.get("region", "center")

        if not obstacle_like:
            action = {
                "action": "idle",
                "priority": "low",
                "reason": "no_obstacle",
            }
            return None, action

        if region == "center":
            action = {
                "action": "stop",
                "priority": "high",
                "reason": "obstacle_center",
            }
            return "Obstacle ahead.", action

        elif region == "left":
            action = {
                "action": "move_suggestion",
                "direction": "right",
                "priority": "normal",
                "reason": "obstacle_left",
            }
            return "Obstacle on the left.", action

        elif region == "right":
            action = {
                "action": "move_suggestion",
                "direction": "left",
                "priority": "normal",
                "reason": "obstacle_right",
            }
            return "Obstacle on the right.", action

        action = {
            "action": "warn",
            "priority": "normal",
            "message": "Obstacle detected",
            "reason": "unknown_region",
        }
        return "Obstacle detected.", action

    def heard_callback(self, msg):
        text = msg.data.strip().lower()

        self.get_logger().info(f"Received user command: {text}")

        if text in ["hello", "hi"]:
            self.quiet_mode = False
            self.say("Hello. Murphy P2 is online.")

        elif text in ["stop", "quiet", "silence"]:
            self.quiet_mode = True
            self.say("Okay. I will stay quiet.")

        elif text in ["speak", "talk", "resume"]:
            self.quiet_mode = False
            self.say("Okay. I will speak again.")

        elif text in ["what do you see", "describe", "describe scene"]:
            self.describe_last_event()

        elif text in ["status", "system status"]:
            self.say("Camera, visual processor, brain, audio, ear, and action nodes are active.")

        elif text.startswith("ask vlm "):
            question = text[len("ask vlm ") :].strip()
            self.ask_vlm(question)

        elif text in ["look carefully", "use vlm", "analyze image"]:
            self.ask_vlm("Describe what you see in the image.")

        else:
            # self.say(f"I heard: {text}")
            self.ask_vlm(text)

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

    def publish_action(self, action):
        """
        Publish structured actions to /brain/actions.

        The action_node can later convert these into motors, LEDs,
        haptics, or navigation commands.
        """
        action_type = action.get("action", "unknown")
        direction = action.get("direction", "")
        reason = action.get("reason", "")

        action_key = f"{action_type}:{direction}:{reason}"

        # Avoid spamming the exact same action every frame
        if action_key == self.last_action_key:
            return

        self.last_action_key = action_key

        msg = String()
        msg.data = json.dumps(action)
        self.action_pub.publish(msg)

        self.get_logger().info(f"Brain action: {msg.data}")

    def ask_vlm(self, question):
        if not question:
            self.say("I did not receive a question for the vision model.")
            return

        if not os.path.exists(self.latest_image_path):
            self.say("I do not have a recent image for the vision model yet.")
            self.get_logger().warn(
                f"Latest image does not exist: {self.latest_image_path}"
            )
            return

        request = {
            "type": "vlm_question",
            "question": question,
            "image_path": self.latest_image_path,
        }

        msg = String()
        msg.data = json.dumps(request)
        self.vlm_question_pub.publish(msg)

        self.last_vlm_question = question

        self.get_logger().info(f"Asked VLM: {msg.data}")

        if not self.quiet_mode:
            self.say("Let me look.")
            
    def vlm_answer_callback(self, msg):
        try:
            response = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Invalid VLM answer JSON: {msg.data}")
            return

        answer = response.get("answer", "").strip()
        success = response.get("success", False)

        if not answer:
            answer = "I did not get an answer from the vision model."

        if not success:
            self.get_logger().warn(f"VLM returned unsuccessful response: {answer}")

        self.get_logger().info(f"VLM answer received: {answer}")

        if not self.quiet_mode:
            self.say(answer)

    def maybe_say_face_event(self, event):
        if self.quiet_mode:
            return

        if not event.get("has_face", False):
            return

        face_coords = event.get("face_coordinates")
        if not face_coords:
            return

        if self.latest_yolo_jpeg_bytes is None:
            self.get_logger().warn("No YOLO compressed image received yet.")
            return

        region = face_coords.get("region", "center")
        confidence = float(face_coords.get("confidence", 0.0))

        if confidence < 0.5:
            return

        if self.face_request_in_flight:
            return

        now = time.time()

        if now - self.last_face_speak_time < self.face_speak_cooldown_sec:
            return

        self.last_face_region = region
        self.last_face_speak_time = now

        prompt = (
            f"The robot sees a person in the {region} of the image. "
            "Look at the image and say one short funny friendly sentence to them. "
            "Do not mention camera, face detection, bounding boxes, confidence, or location. "
            "Keep it under 12 words."
        )

        image_b64 = base64.b64encode(self.latest_yolo_jpeg_bytes).decode("utf-8")

        self.face_request_in_flight = True
        def _send_to_ollama():
            try:
                response = requests.post(
                    self.ollama_vision_url,
                    json={
                        "prompt": prompt,
                        "image_base64": image_b64,
                        "model": "qwen2.5vl:7b",
                        "speak": True,
                    },
                    timeout=180.0,
                )

                if not response.ok:
                    self.get_logger().warn(
                        f"Ollama vision server error: {response.status_code} {response.text}"
                    )
                    return

                self.get_logger().info("Sent YOLO compressed image to Ollama vision server.")

            except Exception as e:
                self.get_logger().warn(f"Could not call Ollama vision server: {e}")

            finally:
                self.face_request_in_flight = False
        
        threading.Thread(target=_send_to_ollama, daemon=True).start()

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