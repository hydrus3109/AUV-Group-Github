#!/usr/bin/env python3

#~/ardupilot/Tools/autotest/sim_vehicle.py --vehicle=ArduSub --aircraft="bwsibot" -L RATBeach --out=udp:YOUUURRR IPPPP:14550
#ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@14555 gcs_url:=udp://:14550@YOUUURRR IPPP:14550 tgt_system:=1 tgt_component:=1 system_id:=255 component_id:=240

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import BatteryState, Imu, FluidPressure

from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

import numpy as np

class ROV_sensors(Node):
    def __init__(self):
        super().__init__("tutorial_subscriber")
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        #BATTERY STUFF
        self.Battery_subcription = self.create_subscription(
            BatteryState,
            "/mavros/battery",
            self.battery_callback,
            qos_profile
        )
        self.battery_data = BatteryState() #Initialize attribute (of type battery status) under the name battery_data
        
        #TIMER TO CHECK FOR BATTERY
        self.publisher_timer = self.create_timer(
            5.0, self.check_safe_voltage #Every second, runs the run_node() method
        )
        
        #IMU STUFF
        self.IMU_sub = self.create_subscription(
            Imu,
            "/mavros/imu/data",
            self.imu_callback,
            qos_profile
        )
        self.imu_data = Imu()
        
        #STATIC PRESSURE STUFF
        self.IMU_Static_P = self.create_subscription(
            FluidPressure,
            "/mavros/imu/static_pressure",
            self.static_p_callback,
            qos_profile
        )
        self.static_p_data = FluidPressure()
        
        #DIFF PRESSURE STUFF
        self.IMU_Diff_P = self.create_subscription(
            FluidPressure,
            "/mavros/imu/diff_pressure",
            self.diff_p_callback,
            qos_profile
        )
        self.diff_p_data = FluidPressure()
        
        self.get_logger().info("starting subscriber node")
    
    def check_safe_voltage(self):
        if self.battery_data.voltage < 12:
            self.get_logger().info("Battery has fallen under safe range")
        
    def battery_callback(self, msg):
        self.battery_data = msg
        self.get_logger().info(f"Battery voltage: {self.battery_data.voltage}") 
    
    def static_p_callback(self,msg):
        self.static_p_data = msg
        self.get_logger().info(f"Static pressure: {self.static_p_data.fluid_pressure}")
        
    def diff_p_callback(self,msg):
        self.diff_p_data = msg
        self.get_logger().info(f"Diff pressure: {self.diff_p_data.fluid_pressure}")
        
    def imu_callback(self, msg): 
        self.imu_data = msg
        self.get_logger().info(f"IMU Linear acceleration: {self.imu_data.linear_acceleration}")
    
def main(args=None):
    rclpy.init(args=args)
    node = ROV_sensors()

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