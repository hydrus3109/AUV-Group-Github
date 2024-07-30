#!/usr/bin/env python3

#~/ardupilot/Tools/autotest/sim_vehicle.py --vehicle=ArduSub --aircraft="bwsibot" -L RATBeach --out=udp:YOUR_COMPUTER_IP:14550
#ros2 launch mavros apm.launch fcu_url:=udp://192.168.2.2:14550@14555 gcs_url:=udp://:14550@YOUR_COMPUTER_IP:14550 tgt_system:=1 tgt_component:=1 system_id:=255 component_id:=240

#cd ~/auvc_ws
#colcon build --symlink-install
#source ~/auvc_ws/install/setup.zsh

#ros2 topic list
#ros2 topic type /your/topic
#ro2 topic echo /your/topic :)))))
#ros2  interface show your_msg_library/msg/YourMessageType

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from mavros_msgs.msg import OverrideRCIn
from std_msgs.msg import Int16, Float32
import numpy as np
import time

FOV_HOR = 80 
FOV_VER = 64 


class LightSubscriber(Node):
    Done = True
    def __init__(self):
        super().__init__("light_subscriber")
        
        self.subscriber = self.create_subscription(
            Float32,
            "bluerov2/distance",
            self.distance_callback,
            10
        )
        self.get_logger().info("starting camera subscriber node")
        
        self.heading_subscriber = self.create_subscription(
            Int16,
            'bluerov2/desired_heading',
            self.heading_callback,
            10
        )
        self.get_logger().info("starting heading subscriber node")
        
        self.move_publisher = self.create_publisher(
            OverrideRCIn,
            "bluerov2/override_rc",
            10
        )
        self.get_logger().info("starting heading publsiher node")

        self.movement = OverrideRCIn()
        self.movement.channels = [65535] * 18
        self.distance = None
        self.x_angle = None


        
    def distance_callback(self, msg):
        """logs and stores float64 distance from subscriber"""
        self.distance = msg.data
        self.turn_on_lights()
        
    def heading_callback(self, msg):
        """logs and stores int16 heading from subscriber"""
        self.x_angle = msg.data   
        
    def turn_on_lights(self):
        """turns on auv lights"""
        self.movement.channels[8] = 2000
        self.movement.channels[9] = 2000    
        self.move_publisher.publish(self.movement)

    def turn_off_lights(self):
        """turns off auv lights"""
        self.movement.channels[8] = 1000
        self.movement.channels[9] = 1000
        self.move_publisher.publish(self.movement)
        
    def flashing_lights(self):
        '''detects if a robot has been seen and flashes lights'''
        if self.x_angle is None:
            return
        if abs(self.x_angle) < 10 and self.distance < 2:
            self.get_logger().info("LIGHTS")
            for i in range (3):
                self.turn_on_lights(self.movement)
                time.sleep(0.2)
                self.turn_off_lights(self.movement)
                time.sleep(0.2)
        self.turn_off_lights(self.movement)
            
            

def main(args=None):
    rclpy.init(args=args)
    node = LightSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()