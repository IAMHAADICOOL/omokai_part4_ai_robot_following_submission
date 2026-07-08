#!/usr/bin/env python3
"""Operator prompt input. Reads natural-language lines from stdin and
publishes them on /mission/prompt. Runs in its own xterm so you can type.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PromptPublisher(Node):
    def __init__(self):
        super().__init__("prompt_publisher")
        self.pub = self.create_publisher(String, "/mission/prompt", 10)
        self.get_logger().info("Interface ready.")

    def run(self):
        print("\n=== Omokai mission console =========================")
        print("Type an instruction and press Enter. Ctrl+D or 'quit' to exit.")
        print("Example: Patrol the perimeter loop twice\n")
        while rclpy.ok():
            try:
                text = input("mission> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if not text:
                continue
            if text.lower() in ("quit", "exit"):
                break
            msg = String()
            msg.data = text
            self.pub.publish(msg)
            self.get_logger().info(f"published prompt: {text!r}")


def main(args=None):
    rclpy.init(args=args)
    node = PromptPublisher()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
