#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Vector3
from mavros_msgs.msg import OverrideRCIn, RCIn, RCOut
from intro_to_ros.arm import ROVController
from intro_to_ros.mode import ModeController
from rclpy.task import Future
import time

class ControlNode(Node):
    def __init__(self, duration):
        super().__init__('control_node')
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        self.thrust_publisher = self.create_publisher(OverrideRCIn, '/mavros/rc/override',qos_profile)
        self.thrust_subscriber = self.create_subscription(RCOut, 'mavros/rc/out', self.thrust_callback, qos_profile)
        self.thrust_subscriber_in = self.create_subscription(RCIn, 'mavros/rc/in', self.thrust_callback_2, qos_profile)
        #self.distance_subscriber = self.create_subscription(Odometry, '/mavros/local_position/odom', self.distance_callback, qos_profile)
        #self.timer = self.create_timer(0.1, self.control_loop)  # Control loop at 10 Hz
        self.get_logger().info("starting publisher node")
        self.start_time = time.time()
        self.duration = duration
        self.timer = self.create_timer(.25, self.open)

        self.run_future = Future()
        self.timer_end = self.create_timer(self.duration, self.complete_future)
    def open(self):
        if not self.run_future.done():
            command = OverrideRCIn()
            command.channels = [65535 for _ in range(18)]
            command.channels[0] = 1500 
            command.channels[1] = 1500 
            command.channels[2] = 1500
            command.channels[3] = 1500 
            command.channels[4] = 2000
            command.channels[7] = 1500
            command.channels[9] = 1500

            self.thrust_publisher.publish(command)
            self.get_logger().info(f"Published Channels: {command.channels}\n")
        else:
            self.get_logger().info("Operation completed. Shutting down...")
            self.destroy_node()
    def complete_future(self):
        self.run_future.set_result(True)
    def thrust_callback(self,msg):
        self.get_logger().info(f"Actual Channels Out: {msg.channels}")
    def thrust_callback_2(self,msg):
        self.get_logger().info(f"Actual Channels In: {msg.channels}")



def main(args=None):
    rclpy.init(args=args)
    control_node = ControlNode(10)
    arm = ROVController()
    mode = ModeController()

    mode.send_mode_command('MANUAL')
    rclpy.spin_once(mode)

    arm.send_arm_command(True)
    rclpy.spin_once(arm)
    
    rclpy.spin_until_future_complete(control_node, control_node.run_future)

    arm.send_arm_command(False)
    
    control_node.destroy_node()
    arm.destroy_node()
    mode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()