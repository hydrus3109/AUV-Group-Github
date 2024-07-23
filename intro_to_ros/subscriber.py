#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

import numpy as np
class TutorialSubscriber(Node):
    def __init__(self):
        super().__init__("tutorial_subscriber")
        self.subscriber = self.create_subscription(
            Vector3,
            "/tutorial/vector3",
            self.callback,
            10
        )
        self.subscriber
        self.get_logger().info("starting subscriber node")
    def callback(self, msg):
        magnitude = np.sqrt(msg.x ** 2 + msg.y ** 2 + msg.z ** 2)
        data = Vector3()
        data.x = msg.x / magnitude
        data.y = msg.y / magnitude
        data.z = msg.z / magnitude
        self.get_logger().info(f"Vector3\n\tx: {data.x}\ty: {data.y}\tz: {data.z}")
def main(args=None):
    rclpy.init(args=args)
    node = TutorialSubscriber()

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