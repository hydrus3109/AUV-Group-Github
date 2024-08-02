#!/usr/bin/env python3

#cd ~/auvc_ws
#colcon build --symlink-install
#source ~/auvc_ws/install/setup.zsh

#ros2 launch /home/kenayosh/auvc_ws/src/AUV-Group-Github/launch/testitall.yaml
#ros2 run intro_to_ros exec

#ros2 topic pub bluerov2/desired_depth mavros_msgs/msg/Altitude "{relative: 0.8}" 
#ros2 topic pub bluerov2/desired_heading std_msgs/msg/Int16 "{data: 3}" 
#ros2 topic echo /your/topic 

from enum import Enum, auto
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, Range
from mavros_msgs.msg import ManualControl, Altitude, OverrideRCIn
from std_msgs.msg import Int16, Float32, Bool
import numpy as np
import time

class State(Enum):
    PREGAME = auto()
    SCANNING = auto()
    CHASE = auto()
    FLASHING = auto()

class StateBetter(Node):
    MIN_DEPTH = 0.3                          #Minimum depth we can dive
    MAX_DEPTH = 0.5                           #Maximum depth we can dive
    MIN_DISTANCE_TO_OPPONENT = 2                #Distance to opponent before we can shoot
    START_HEADING = 90
    def __init__(self):
        super().__init__('state_better')
        self.IMG_heading_subscriber     = self.create_subscription(Int16   ,'img/desired_heading', self.IMG_heading_callback, 10)
        self.AT_distance_subscriber     = self.create_subscription(Float32  ,"img/distance", self.AT_distance_callback, 10) #distance to the AprilTag
        self.targetted_subscriber       = self.create_subscription(Bool     ,'img/targetted', self.targetted_callback, 10) #if both AprilTag and YOLO fails
        self.more_lanes_subscriber      = self.create_subscription(Bool   ,'ld/more_lanes', self.more_lanes_callback, 10) #check if there are more lanes to get ready to turn
        self.lateral_offset_subscriber  = self.create_subscription(Float32  ,'ld/lateral_offset', self.lateral_offset_callback, 10) #how far are we to the lane
        
        self.direct_manual_publisher        = self.create_publisher(ManualControl,'bluerov2/manual_control',10) #just for x and y
        self.PID_desired_depth_publisher    = self.create_publisher(Altitude, 'PID/desired_depth', 10) #just for lights
        self.PID_desired_heading_publisher  = self.create_publisher(Int16,'PID/desired_heading', 10)

        self.target_found = False  #OMAKE SURE FALSE
        self.IMG_heading = None
        self.distance_to_opponent = None 
        self.lateral_offset = None
        self.lane_found = False #make sure this is set to true
        self.current_depth = None
        self.lights_on = 10
        self.current_state = State.CHASE
        self.scan_counter = 0

    def IMG_heading_callback(self, msg):
        if msg is not None:
            self.IMG_heading = int(msg.data)     
    def AT_distance_callback(self, msg):
        if(self.target_found):
            self.distance_to_opponent = msg.data
        else:
            self.distance_to_opponent = None

    def lateral_offset_callback(self, msg):
        self.lateral_offset = msg.data          #offset from the lanes

    def more_lanes_callback(self, msg):        
        self.lane_found = msg.data 

    def targetted_callback(self,msg):   
        if(msg.data):
            self.target_found = True
        else:
            self.target_found = False 

    def pregame(self):                          #Assuming we can place our robot not head-on with the opponent
        depth_msg = Altitude()
        depth_msg.relative = self.MAX_DEPTH
        self.PID_desired_depth_publisher.publish(depth_msg)

        msg = Int16()
        msg.data = self.START_HEADING                     #Angle of the lanes (measured)
        self.PID_desired_heading_publisher.publish(msg)

        movement = ManualControl()
        movement.x = 100.0                        # Move forward with the lanes
        self.direct_manual_publisher.publish(movement)

        # if self.lateral_offset is not None:
        #     movement.y = min(self.lateral_offset, 20)
        #     self.get_logger().info(f'\nCurrent Power for lateral offset: {movement.y}')
        #     self.direct_manual_publisher.publish(movement)
    
    def scan(self):
        """scanning behaviour for finding other robot"""
        #moves forward slowly
        if (self.scan_counter <= 500):
            movement = ManualControl()
            movement.x = 20.0
            self.direct_manual_publisher.publish(movement)
        
        desired_depth = Altitude()
        desired_depth.relative = self.MAX_DEPTH   # We want to go to the maximum possible depth in our range
        self.PID_desired_depth_publisher.publish(desired_depth)
        
        newheading = Int16()
        newheading.data = (self.START_HEADING + 180) % 360        
        self.PID_desired_heading_publisher.publish(newheading)
        
        # if self.lateral_offset is not None:     #make sure we stick to the lane
        #     movement.y = min(self.lateral_offset, 20)
        #     self.get_logger().info(f'\nCurrent Power for lateral offset: {movement.y}')
        #     self.direct_manual_publisher.publish(movement)
        # self.direct_manual_publisher.publish()
        
        self.scan_counter += 1
        #self.get_logger().info(f"{self.scan_counter}")
        
        if (self.scan_counter > 500):
            movement = ManualControl()
            movement.x = 0.0001
            self.direct_manual_publisher.publish(movement)
        
            if self.scan_counter > 2000:
                self.scan_counter = 0
        
        
        # self.get_logger().info(f"target found: {self.target_found}")
            

    def chase(self):
        '''Implement moving behavior towards the opponent'''
        self.get_logger().info("CHASE")
        newheading = Int16()                    # sets our PID heading to the angle we detected the AprilTag/model
        if self.IMG_heading is not None:
            newheading.data = self.IMG_heading
            self.PID_desired_heading_publisher.publish(newheading)

        msg = Altitude()
        msg.relative = (self.MAX_DEPTH + self.MIN_DEPTH)/2
        self.PID_desired_depth_publisher.publish(msg)

        # Moving behavior towards the opponent
        movement = ManualControl()
        movement.x = 30.0  # Move forward
        self.direct_manual_publisher.publish(movement)

        #set heading according to what is found from camera subscriber
        newheading = Int16()
        # if(self.IMG_heading is None):
        #     self.get_logger().info("[ERR:COOKED] IDK WHATS HAPPENING BUT TARGET HEADING IS NONE IN THE CHASE FUNCTIONNNN")
        #     return
        newheading.data = self.IMG_heading
        self.PID_desired_heading_publisher.publish(newheading)
        


    def flash(self):
        """Flashes lights at the AUV"""
        #Maintains middle depth
        RCmovement = OverrideRCIn()
        msg = Altitude()
        msg.relative = (self.MAX_DEPTH + self.MIN_DEPTH)/2
        self.PID_desired_depth_publisher.publish(msg)

        #sets up a timer, lights_on is a counter, each 10 frames the light either turns on or turns off
        self.lights_on += 1
        if(self.lights_on > 100):
            RCmovement.channels[8] = 2000
            RCmovement.channels[9] = 2000   
            self.get_logger().info("LIGHTS ON") 
            self.direct_lights_publisher.publish(RCmovement)
        elif(self.lights_on > 200):
            self.lights_on = 0
            RCmovement.channels[8] = 1000
            RCmovement.channels[9] = 1000
            self.direct_lights_publisher.publish(RCmovement)
            
        
        self.direct_lights_publisher.publish(RCmovement)
        #make sure that we stop when we're flashing the lights
        ManualMovement = ManualControl()
        ManualMovement.x = 0.0
        self.direct_manual_publisher.publish(ManualMovement)
        

    def perform_action(self):
        if self.current_state == State.PREGAME:
            self.pregame()
        elif self.current_state == State.SCANNING:
            self.scan()
        elif self.current_state == State.CHASE:
            self.chase()
        elif self.current_state == State.FLASHING:
            self.flash()
            
    
    def update_state(self):
        #self.get_logger().info(f"Lane bool: {self.lane_found}")
        """Handles state based transitions"""
        # Update the current state based on sensor data
        if self.current_state == State.PREGAME:
            if not self.lane_found:
                #Turns the AUV around by setting desired heading
                msg = Int16()
                msg.data = (self.START_HEADING + 180) % 360
                self.PID_desired_heading_publisher.publish(msg)

                self.get_logger().info("---ENTERING SCANNING STAGE---")
                self.current_state = State.SCANNING

        # Helps transition from scanning state to chasing state
        if self.current_state == State.SCANNING:
            if self.target_found:
                self.get_logger().info("---FOUND AUV, ENTERING CHASING STAGE---")
                self.current_state = State.CHASE

        # Helps transition from chasing state back to scanning state if auv lost or to flashing state if auv distance is close
        elif self.current_state == State.CHASE:
            if self.distance_to_opponent is None:
                self.get_logger().info("---AUV LOST, ENTERING BACK TO SCANNING STAGE---")
                self.current_state = State.SCANNING
            elif self.distance_to_opponent < self.MIN_DISTANCE_TO_OPPONENT:
                self.get_logger().info("---AUV CLOSE, FLASHING LIGHTS---")
                self.current_state = State.FLASHING

        # Transitions from flashing state back to chasing state if AUV distance is too big or AUV lost
        elif self.current_state == State.FLASHING:
            if self.distance_to_opponent is None and self.distance_to_opponent > self.MIN_DISTANCE_TO_OPPONENT:
                self.get_logger().info("---AUV ESCAPED, ENTERING BACK TO CHASING STAGE---")
                self.current_state = State.CHASE
    
    def run(self):
        self.get_logger().info("RUN")
        while rclpy.ok():
            self.update_state()
            self.perform_action()

def main(args=None):
    rclpy.init(args=args)
    node = StateBetter()

    try:
        node.run()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.RCmovement.channels[8] = 1000
        node.RCmovement.channels[9] = 1000
        node.direct_lights_publisher.publish(node.RCmovement)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()