import base64
import json
import os
import urllib.request
import urllib.error

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VLMNode(Node):
    def __init__(self):
        super().__init__("vlm_node")

        # self.declare_parameter("ollama_url", "http://host.docker.internal:11434")
        self.declare_parameter("ollama_url", "http://192.168.68.59:11434")
        self.declare_parameter("model", "moondream")
        self.declare_parameter("timeout_sec", 90.0)

        self.ollama_url = self.get_parameter("ollama_url").value.rstrip("/")
        self.model = self.get_parameter("model").value
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)

        self.question_sub = self.create_subscription(
            String,
            "/vlm/questions",
            self.question_callback,
            10,
        )

        self.answer_pub = self.create_publisher(
            String,
            "/vlm/answers",
            10,
        )

        self.get_logger().info("VLM node started.")
        self.get_logger().info(f"Ollama URL: {self.ollama_url}")
        self.get_logger().info(f"Model: {self.model}")

    def question_callback(self, msg):
        try:
            request = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Invalid JSON request: {msg.data}")
            return

        question = request.get("question", "").strip()
        image_path = request.get("image_path", "").strip()

        if not question:
            self.publish_answer(
                question="",
                answer="I did not receive a question.",
                image_path=image_path,
                success=False,
            )
            return

        if not image_path:
            self.publish_answer(
                question=question,
                answer="I do not have an image path to inspect.",
                image_path=image_path,
                success=False,
            )
            return

        if not os.path.exists(image_path):
            self.publish_answer(
                question=question,
                answer=f"I could not find the image file: {image_path}",
                image_path=image_path,
                success=False,
            )
            return

        self.get_logger().info(f"Received VLM question: {question}")
        self.get_logger().info(f"Using image: {image_path}")

        try:
            answer = self.ask_ollama(question, image_path)
            self.publish_answer(
                question=question,
                answer=answer,
                image_path=image_path,
                success=True,
            )
        except Exception as exc:
            self.get_logger().error(f"Ollama request failed: {exc}")
            self.publish_answer(
                question=question,
                answer="I had trouble using the visual language model.",
                image_path=image_path,
                success=False,
            )

    def ask_ollama(self, question, image_path):
        image_b64 = self.encode_image(image_path)

        prompt = (
            f"Question: {question}"
        )

        payload = {
            "model": self.model,
            "prompt": prompt,
            "images": [image_b64],
            "stream": False,
            "options": {
                "temperature": 0.0,
                "num_predict": 100,
            },
        }

        data = json.dumps(payload).encode("utf-8")

        req = urllib.request.Request(
            url=f"{self.ollama_url}/api/generate",
            data=data,
            headers={"Content-Type": "application/json"},
            method="POST",
        )

        with urllib.request.urlopen(req, timeout=self.timeout_sec) as response:
            raw = response.read().decode("utf-8")
            # self.get_logger().info(f"Raw Ollama response: {raw[:1000]}")

        result = json.loads(raw)
        answer = result.get("response", "").strip()

        if not answer or len(answer) < 2:
            return "I could not get a useful answer from the vision model."

        return answer

    def encode_image(self, image_path):
        with open(image_path, "rb") as f:
            image_bytes = f.read()

        return base64.b64encode(image_bytes).decode("utf-8")

    def publish_answer(self, question, answer, image_path, success):
        response = {
            "type": "vlm_answer",
            "question": question,
            "answer": answer,
            "image_path": image_path,
            "success": success,
            "model": self.model,
        }

        msg = String()
        msg.data = json.dumps(response)
        self.answer_pub.publish(msg)

        self.get_logger().info(f"VLM answer: {answer}")


def main(args=None):
    rclpy.init(args=args)
    node = VLMNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()