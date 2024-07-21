#!/usr/bin/env python3

import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Pose2D

import numpy as np

class OneSubscriber(Node):
    def __init__(self):
        super().__init__("physics_subscriber")
        self.subscriber = self.create_subscription(
            Pose2D,
            "/physics/pose2d",
            self.callback_pose2d,
            QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        )
        
        self.pose2d = Pose2D()
        self.subscriber
        self.get_logger().info("starting subscriber node")

    def pose2d_callback(self, msg):
        self.battery = msg
        #self.get_logger().info(f"Battery %: {self.battery.percentage}\nBattery Voltage {self.battery.voltage}")
        simulate_auv_motion(msg.x,msg.y,msg.z)
   

def main(args=None):
    rclpy.init(args=args)
    node = OneSubscriber()

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