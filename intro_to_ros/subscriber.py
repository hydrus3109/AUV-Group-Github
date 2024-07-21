#!/usr/bin/env python3

import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Imu, FluidPressure
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from mavros_msgs.msg import Mavlink
from struct import pack, unpack

import numpy as np
#https://rawgithubusercontent.com/mavlink/mavros/blob/ros2/mavros/launch/px4_config.yaml.
class OneSubscriber(Node):
    def __init__(self):
        super().__init__("subscriber")
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE)
        # self.subscriber = self.create_subscription(
        #     BatteryState,
        #     "/mavros/battery",
        #     self.callback_battery,
        #     qos_profile
        # )
        self.subscriber_2 = self.create_subscription(
            Imu,
            "/mavros/imu/data",
            self.callback_imu,
            qos_profile
        )
        
        # self.subscriber_3 = self.create_subscription(
        #     FluidPressure,
        #     "/mavros/imu/static_pressure",
        #     self.callback_pressure,
        #     qos_profile
        # )

        self.subscriber_4 = self.create_subscription(
            Mavlink,
            "mavros/mavlink",
            self.callback_mav_pressure,
            qos_profile
        )
        
        self.imu = Imu()
        self.battery = BatteryState()
        self.get_logger().info("starting subscriber node")
        self.timer = self.create_timer(5, self.check_battery)
        self.pressure = 0

    def callback_battery(self, msg):
        self.battery = msg
        #self.get_logger().info(f"Battery %: {self.battery.percentage}\nBattery Voltage {self.battery.voltage}")

    def callback_imu(self, msg):
        self.imu = msg
        #self.get_logger().info(f"Imu\nOrientation Covariance: {self.imu.orientation_covariance}\nVelocity Covariance: {self.imu.angular_velocity_covariance}\nAcceleration Covariance: {self.imu.linear_acceleration_covariance}\n")
        #self.get_logger().info(f"Imu\nOrientation: {self.imu.linear_acceleration}\n")
    
    def callback_pressure(self, msg):
        self.get_logger().info(f"Static Pressure: {msg}\n")

    def check_battery(self):
        if self.battery.voltage < 50:
            self.get_logger().info("Battery has fallen below safe voltage!")  

    def callback_mav_pressure(self, msg):
        # Check if message id is valid (I'm using SCALED_PRESSURE
        # and not SCALED_PRESSURE2)
        self.get_logger().info(f"Real Pressure\n")
        if msg.msgid == 137:
            rclpy.loginfo(rclpy.get_caller_id() + " Package: %s", msg)
            # Transform the payload in a python string
            p = pack("QQ", *msg.payload64)
            # Transform the string in valid values
            # https://docs.python.org/2/library/struct.html
            time_boot_ms, press_abs, press_diff, temperature = unpack("Iffhxx", p)
        self.pressure = press_abs
        self.get_logger().info(f"Real Pressure: {press_abs}\n")
      

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