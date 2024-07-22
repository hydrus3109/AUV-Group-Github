#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

import random
class TutorialPublisher(Node):
    def __init__(self):
        super().__init__("tutorial_publisher")
        self.publisher = self.create_publisher(
            Vector3,
            "/tutorial/vector3",
            10
        )
        self.publisher_timer = self.create_timer(
            1.0, self.run_node
        )
        self.get_logger().info("starting publisher node")
    def run_node(self):
        msg = Vector3()
        msg.x = random.uniform(-10.0, 10.0)
        msg.y = random.uniform(-10.0, 10.0)
        msg.z = random.uniform(-10.0, 10.0)
        self.publisher.publish(msg)
        self.get_logger().info(f"Vector3\n\tx: {msg.x}\ty: {msg.y}\tz: {msg.z}")
def main(args=None):
    rclpy.init(args=args)
    node = TutorialPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__=="__main__":
    main()