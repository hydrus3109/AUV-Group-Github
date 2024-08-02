#!/usr/bin/env python3

#cd ~/auvc_ws
#colcon build --symlink-install
#source ~/auvc_ws/install/setup.zsh

#ros2 launch /home/kenayosh/auvc_ws/src/AUV-Group-Github/launch/testitall.yaml
#ros2 run intro_to_ros exec

#ros2 topic pub bluerov2/desired_depth mavros_msgs/msg/Altitude "{relative: 0.8}" 
#ros2 topic pub img/desired_heading std_msgs/msg/Int16 "{data: 3}" 
#ros2 topic echo /your/topic 
#ros2 topic pub img/targetted std_msgs/msg/Bool "{data: True}"

from enum import Enum, auto
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ManualControl, Altitude, OverrideRCIn
from std_msgs.msg import Int16, Float32, Bool
import numpy as np
import time

class State(Enum):
    SCANNING = auto()
    CHASE = auto()

class Movement(Node):
    MIN_DEPTH = 0.3                          #Minimum depth we can dive
    MAX_DEPTH = 0.8                           #Maximum depth we can dive
    MIN_DISTANCE_TO_OPPONENT = 1                #Distance to opponent before we can shoot
    START_HEADING = 90
    def __init__(self):
        super().__init__('state_better')
        self.IMG_heading_subscriber     = self.create_subscription(Int16   ,'img/desired_heading', self.IMG_heading_callback, 10)
        self.AT_distance_subscriber     = self.create_subscription(Float32  ,"img/distance", self.AT_distance_callback, 10) #distance to the AprilTag
        self.targetted_subscriber       = self.create_subscription(Bool     ,'img/targetted', self.targetted_callback, 10) #if both AprilTag and YOLO fails
        # self.robot_heading_subscriber   = self.create_subscription(Int16    ,'bluerov2/heading', self.robot_heading_callback)        
                
        self.direct_manual_publisher        = self.create_publisher(ManualControl,'bluerov2/manual_control',10) #just for x and y
        self.direct_lights_publisher        = self.create_publisher(OverrideRCIn,'bluerov2/override_rc',10) #just for lights
        self.PID_desired_depth_publisher    = self.create_publisher(Altitude, 'PID/desired_depth', 10) #
        self.PID_desired_heading_publisher  = self.create_publisher(Int16,'PID/desired_heading', 10)

        self.target_found = False  #OMAKE SURE FALSE
        self.IMG_heading = None
        self.distance_to_opponent = None 
        self.current_depth = None
        self.current_state = State.SCANNING
        self.scan_counter = 0
        self.robot_heading = None
        
        self.publisher_timer = self.create_timer(0.2, self.run)
        
    def IMG_heading_callback(self, msg):
        if msg is not None:
            self.IMG_heading = int(msg.data)   
        else:
            self.IMG_heading = None
            
    # def robot_heading_callback(self, msg):
    #     if msg is not None:
    #         self.robot_heading = int(msg.data)   
    #     else:
    #         self.robot_heading = None
             
    def AT_distance_callback(self, msg):
        self.get_logger().info("AT distance callback")
        if(self.target_found):
            self.distance_to_opponent = msg.data
        else:
            self.distance_to_opponent = None
        self.get_logger().info(f"distance : {self.distance_to_opponent}")
        if(self.distance_to_opponent is not None and self.distance_to_opponent < self.MIN_DISTANCE_TO_OPPONENT):
            self.flash()

    def targetted_callback(self, msg):
        self.target_found = msg.data

    def scan(self):
        """scanning behaviour for finding other robot"""
        #moves forward slowly
        if (self.scan_counter <= 10):
            movement = ManualControl()
            movement.x = 20.0
            self.direct_manual_publisher.publish(movement)
        
        desired_depth = Altitude()
        desired_depth.relative = self.MIN_DEPTH   # We want to go to the maximum possible depth in our range
        self.PID_desired_depth_publisher.publish(desired_depth)
        
        newheading = Int16()
        newheading.data = (self.START_HEADING)       
        self.PID_desired_heading_publisher.publish(newheading)
        
        self.scan_counter += 1
        #self.get_logger().info(f"{self.scan_counter}")
        
        if (self.scan_counter > 10):
            movement = ManualControl()
            movement.x = 0.0001
            self.direct_manual_publisher.publish(movement)
        
            if self.scan_counter > 50:
                self.scan_counter = 0

    def chase(self):
        '''Implement moving behavior towards the opponent'''
                            # sets our PID heading to the angle we detected the AprilTag/model
        if self.IMG_heading is not None:
            newheading = Int16()
            newheading.data = int(self.IMG_heading)
            self.PID_desired_heading_publisher.publish(newheading)
            
            # Moving behavior towards the opponent
            movement = ManualControl()
            movement.x = 30.0  # Move forward
            self.direct_manual_publisher.publish(movement)
        # else:
        #     newheading = Int16()
        #     newheading.data = self.robot_heading + 1    
        #     self.PID_desired_heading_publisher.publish(newheading)
            
        msg = Altitude()
        msg.relative = (self.MAX_DEPTH + self.MIN_DEPTH)/2
        self.PID_desired_depth_publisher.publish(msg)


    def flash(self):
        """Flashes lights at the AUV"""
        #Maintains middle depth
        self.get_logger().info("LIGHTS CALLED") 

        #sets up a timer, lights_on is a counter, each 10 frames the light either turns on or turns off
        
        RCmovement = OverrideRCIn()
        
        for i in range(5):
            RCmovement.channels[8] = 2000
            RCmovement.channels[9] = 2000 
            self.direct_lights_publisher.publish(RCmovement)
            
            time.sleep(0.5)
            
            RCmovement.channels[8] = 1000
            RCmovement.channels[9] = 1000 
            self.direct_lights_publisher.publish(RCmovement)
            
            time.sleep(0.5)
        
        self.direct_lights_publisher.publish(RCmovement)
        #make sure that we stop when we're flashing the lights 

    def perform_action(self):
        if self.current_state == State.SCANNING:
            self.scan()
        elif self.current_state == State.CHASE:
            self.chase()
            
    
    def update_state(self):
        # Helps transition from scanning state to chasing state
        if self.current_state == State.SCANNING:
            self.get_logger().info("SCANNING STAGE")
            if self.target_found:
                self.get_logger().info("---FOUND AUV, ENTERING CHASING STAGE---")
                self.current_state = State.CHASE

        # Helps transition from chasing state back to scanning state if auv lost or to flashing state if auv distance is close
        elif self.current_state == State.CHASE:
            self.get_logger().info("CHASE STAGE")
    
    def run(self):
        self.update_state()
        self.perform_action()

def main(args=None):
    rclpy.init(args=args)
    node = Movement()

    try:
        # node.run()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        # RCmovement = OverrideRCIn()
        # RCmovement.channels[8] = 1000
        # RCmovement.channels[9] = 1000
        # node.direct_lights_publisher.publish(RCmovement)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()