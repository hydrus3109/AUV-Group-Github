#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
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
        self.subscriber = self.create_subscription(
            Pressure,
            "bluerov2/pressure",
            self.pressure_callback,
            10
        )
        
        self.subscriber = self.create_subscription(
            Temperature,
            "bluerov2/temperature",
            self.temperature_callback,
            10
        )
        
        self.get_logger().info("starting subscriber node")
        
    def temperature_callback(self, msg):
        self.temperature = msg.temperature
        self.get_logger().info(f"Temperature: {self.temperature}")
        
        
    def pressure_callback(self, msg):
        self.pressure = msg.fluid_pressure
        self.depth = self.depth(self.pressure)
        self.get_logger().info(f"Pressure: {self.pressure}, Depth: {self.depth}")
        
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