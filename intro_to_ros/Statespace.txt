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
    MIN_DEPTH = 0.3
    MAX_DEPTH = 0.8
    DISTANCE_TO_OPP = 2
    
    def __init__(self):
        super().__init__('auv_controller')
        self.camera_subscriber = self.create_subscription(Image, 'camera/image_raw', self.camera_callback, 10)
        self.depth_subscriber = self.create_subscription(
            Altitude,
            'bluerov2/depth',
            self.depth_callback,
            10
        )
        self.move_publisher = self.create_publisher(
            ManualControl,
            'bluerov2/manual_control',
            10
        )
        self.heading_subscriber = self.create_subscription(
            Int16,
            'bluerov2/heading',
            self.heading_callback,
            10
        )      
        self.distance_subscriber = self.create_subscription(
            Float32,
            "bluerov2/distance",
            self.distance_callback,
            10
        )

        self.targetted_subscriber = self.create_subscription(
            Bool,
            'bluerov2/targetted',
            self.targetted_callback,
            10
        )

        self.desired_heading_subscriber = self.create_subscription(
            Int16,
            self.desired_heading_callback,
            10
        )

        self.more_lanes_subscriber = self.create_subscription(
            Int16,
            'bluerov/more_lanes',
            self.more_lanes_callback,
            10
        )

        self.lateral_offset_subscriber = self.create_subscription(
            Float32,
            'bluerov2/lateral_offset',
            self.lateral_offset_callback,
            10
        )
        
        #till here: heading and lane number subscriber
        self.move_publisher = self.create_publisher(                    #Initialize the publisher
            OverrideRCIn, #Type of message that's boreadcasted
            "bluerov2/override_rc", #Topic name
            10
        )
        self.desired_depth_publisher = self.create_publisher(
            Altitude,
            'bluerov2/desired_depth',
            10
        )
        self.desired_heading_publisher = self.create_publisher(
            Int16,
            'bluerov2/desired_heading'
            10
        )
        
        
        self.start_heading = None
        self.current_state = State.PREGAME
        self.current_heading = None
        self.current_depth = None
        self.distance_to_opponent = None
        self.relative_heading_to_opponent = None
        self.depth_difference = None
        self.target_found = False
        self.lane_found = False         
        self.lines_detected = False     #
        self.lane_found = False         #
        self.lateral_offset = None      #Lateral motor movement

    def lateral_offset_callback(self, msg):
        self.lateral_offset = msg.data

    def distance_callback(self, msg):
        if(self.target_found):
            self.distance_to_opponent = msg.data
        else:
            self.distance_to_opponent = None
    def camera_callback(self, msg):
        # Process image to detect the opponent and update distance and relative heading
        # edit this to make sure that it takes desired heading when an apriltag has been found
        self.relative_heading_to_opponent=self.desired_heading
        pass
        self.update_state()

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

    def perform_action(self):
        if self.current_state == State.PREGAME:
            self.pregame()
        elif self.current_state == State.SCANNING:
            self.scan()
        elif self.current_state == State.CHASE:
            self.chase()
        elif self.current_state == State.FLASHING:
            self.flash()

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
    def scan(self):
        """scanning behaviour for finding other robot"""
        #maintain low depth
        self.depth_real = Altitude()
        self.depth_real.relative = 0.9
        self.desired_depth_publisher.publish(self.depth_real)
        newheading = Int16()
        newheading.data = self.start_heading + 10
        self.desired_heading_publisher.publish(newheading)
        time.sleep(1)
        newheading.data = self.start_heading- 10
        self.desired_depth_publisher.publish(newheading)
        time.sleep(1)
        newheading.data = self.start_heading
        self.desired_heading_publisher.publish(newheading)
        time.sleep(1)
        cmd = ManualControl()
        cmd.x = 50  
        self.move_publisher.publish(cmd)
        time.sleep(1)
        cmd.x = 0
        self.move_publisher.publish(cmd)

    def chase(self):
        # Implement moving behavior towards the opponent
        cmd = ManualControl()
        cmd.x = 50  # Move forward
        newheading = Int16()
        newheading.data = self.relative_heading_to_opponent
        self.desired_heading_publisher.publish(newheading)
        self.manual_control_publisher.publish(cmd)
        
    def flash(self, movement):
        for i in range (3):
            movement.channels[8] = 2000
            movement.channels[9] = 2000    
            self.move_publisher.publish(movement)
            time.sleep(0.2)
            movement.channels[8] = 1000
            movement.channels[9] = 1000
            self.move_publisher.publish(movement)
            time.sleep(0.2)
        self.turn_off_lights(self.movement)
        self.get_logger().info("Flashing light!")
        #make sure that we stop when we're flashing thhe lights
        cmd = ManualControl()
        cmd.x = 0
        self.manual_control_publisher.publish(cmd)

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            self.update_state()
            self.perform_action()

def main(args=None):
    rclpy.init(args=args)
    node = AUVController()

    try:
        node.run()
        # rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()





