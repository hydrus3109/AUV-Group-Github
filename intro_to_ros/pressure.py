#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import (
    FluidPressure as Pressure,
    Temperature,
)

FLUID_DENSITY = 1000
GRAVITATION_CONST = 9.81
ONEATM_TO_PASCAL = 101325

import numpy as np
class SensorSubscriber(Node):
    def __init__(self):
        super().__init__("sensor_subscriber")
        self.pressure_subscriber = self.create_subscription(
            Pressure,
            "bluerov2/pressure",
            self.pressure_callback,
            10
        )
        
        self.temp_subscriber = self.create_subscription(
            Temperature,
            "bluerov2/temperature",
            self.temperature_callback,
            10
        )
        
        self.depth_publisher = self.create_publisher(
            Float32,
            "/bluerov2/depth",
            10
        )
        
        self.get_logger().info("starting subscriber node")
        
    def temperature_callback(self, msg):
        self.temperature = msg.temperature
        self.get_logger().info(f"Temperature: {self.temperature}")
        
        
    def pressure_callback(self, msg):
        self.depth_real = Float32()
        self.depth_real.data = self.depth(msg.fluid_pressure)
        self.depth_publisher.publish(self.depth_real)

        self.get_logger().info(f"Pressure: {msg.fluid_pressure}, Depth: {self.depth_real.data}")
        
    def depth(self, pressure):
        """Converts from pressure in Pa to Depth
        """
        return (pressure-ONEATM_TO_PASCAL)/GRAVITATION_CONST/FLUID_DENSITY
        
def main(args=None):
    rclpy.init(args=args)
    node = SensorSubscriber()

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