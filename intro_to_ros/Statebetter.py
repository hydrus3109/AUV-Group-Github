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

class AUVController(Node):
    MIN_DEPTH = 0.3                             #Minimum depth we can dive
    MAX_DEPTH = 0.8                             #Maximum depth we can dive
    MIN_DISTANCE_TO_OPPONENT = 2                #Distance to opponent before we can shoot
    def __init__(self):
        super().__init__('auv_controller')
        self.AT_heading_subscriber      = self.create_subscription(Int16    ,'img/desired_heading_AT', self.AT_heading_callback, 10)
        self.Yolo_heading_subscriber    = self.create_subscription(Int16    ,'img/desired_heading_YOLO', self.Yolo_heading_callback, 10)
        self.AT_distance_subscriber     = self.create_subscription(Float32  ,"img/distance", self.AT_distance_callback, 10) #distance to the AprilTag
        self.targetted_subscriber       = self.create_subscription(Bool     ,'img/targetted', self.targetted_callback, 10) #switching between AprilTag and YOLO models
        self.more_lanes_subscriber      = self.create_subscription(Int16    ,'ld/more_lanes', self.more_lanes_callback, 10) #check if there are more lanes to get ready to turn
        self.lateral_offset_subscriber  = self.create_subscription(Float32  ,'ld/lateral_offset', self.lateral_offset_callback, 10) #how far are we to the lane
        self.Lane_heading_subscriber    = self.create_subscription(Int16    ,'ld/desired_heading_LANE', self.Lane_callback, 10)
        self.Robot_heading_subscriber   = self.create_subscription(Int16    ,'bluerov2/heading', self.Robot_heading_callback, 10)


        self.manual_publisher           = self.create_publisher(ManualControl,'bluerov2/manual_control',10) #just for x and y
        self.lights_publisher           = self.create_publisher(OverrideRCIn,'bluerov2/override_rc',10) #just for lights
        self.desired_depth_publisher    = self.create_publisher(Altitude, 'PID/desired_depth', 10) #just for lights
        self.desired_heading_publisher  = self.create_publisher(Int16,'PID/desired_heading', 10)
        
        self.target_found = False
        self.start_heading = None
        
    def lateral_offset_callback(self, msg):
        self.lateral_offset = msg.data

    def AT_distance_callback(self, msg):
        if(self.target_found):
            self.distance_to_opponent = msg.data
        else:
            self.distance_to_opponent = None

    def desired_heading_callback(self, msg):
        self.desired_heading = msg.data

    def more_lanes_callback(self, msg):
        self.lane_found = msg.data
        
    def desired_heading_callback(self, msg):
        if (msg.data is not None):
            self.desired_heading = msg.data       
    
    def depth_callback(self, msg):
        self.current_depth = msg.relative

    def heading_callback(self, msg):
        if self.start_heading is None:
            self.start_heading = msg.data
        self.current_heading = msg.data

    def targetted_callback(self,msg):
        if(msg.data):
            self.target_found = True
        else:
            self.target_found = False

    def pregame(self):
        START_HEADING = 60
        self.desired_heading_publisher.publish(START_HEADING)
        # Implement moving behavior towards the opponent
        cmd = ManualControl()
        cmd.x = 100  # Move forward
        if self.lateral_offset is not None:
            movement = ManualControl()
            movement.y = min(self.lateral_offset, 20)
            self.get_logger().info(f'\nCurrent Power for lateral offset: {movement.y}')
            self.move_publisher.publish(movement)
        if True:
            #DO THE FKING TURNAROUND S

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
        # Update the current state based on sensor data
        if self.current_state == State.PREGAME:
            if not self.lane_found:
                self.current_state = State.SCANNING

        if self.current_state == State.SCANNING:
            if self.distance_to_opponent is not None and not self.target_found:
                self.current_state = State.CHASE

        elif self.current_state == State.CHASE:
            if self.distance_to_opponent is None:
                self.current_state = State.SCANNING

        elif self.current_state == State.FLASHING:
            if self.distance_to_opponent is None and self.distance_to_opponent > 5.0:  
                self.current_state = State.CHASE

        