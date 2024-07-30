from enum import Enum, auto
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, Range
from mavros_msgs.msg import ManualControl, Altitude, OverrideRCIn
from std_msgs.msg import Int16
import numpy as np
import time

class State(Enum):
    SCANNING = auto()
    MOVING = auto()
    ORBITING = auto()
    FLASHING = auto()

class AUVController(Node):
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
            10
        )
        self.start_heading = None
        self.current_state = State.SCANNING
        self.current_heading = None
        self.current_depth = None
        self.distance_to_opponent = None
        self.relative_heading_to_opponent = None
        self.retreat_countdown = 60
        self.depth_difference = None

    def camera_callback  (self, msg):
        # Process image to detect the opponent and update distance and relative heading
        self.distance_to_opponent=self.distance_to_opponent
        self.relative_heading_to_opponent=self.relative_heading_to_opponent
        pass
        if self.distance_to_opponent < 2: #replace this 
            self.flash()
        
        

    def depth_callback(self, msg):
        self.current_depth = msg.relative

    def heading_callback(self, msg):
        if self.start_heading is None:
            self.start_heading = msg.data
        self.current_heading = msg.data

    def update_state(self):
        # Update the current state based on sensor data
        if self.current_state == State.SCANNING:
            if self.distance_to_opponent is not None:
                self.current_state = State.MOVING
        elif self.current_state == State.MOVING:
            if self.distance_to_opponent < 5.0:  
                self.current_state = State.ORBITING
        elif self.current_state == State.ORBITING:
            if self.distance_to_opponent < 2.0:  
                self.current_state = State.FLASHING
        elif self.current_state == State.FLASHING:
            if self.distance_to_opponent > 2.0:
                self.current_state = State.ORBITING

    def perform_action(self):
        if self.current_state == State.SCANNING:
            self.scan()
        elif self.current_state == State.MOVING:
            self.move()
        elif self.current_state == State.ORBITING:
            self.orbit()
        elif self.current_state == State.FLASHING:
            self.flash()

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

    def move(self):
        # Implement moving behavior towards the opponent
        cmd = ManualControl()
        cmd.x = 50  # Move forward
        newheading = Int16()
        newheading.data = self.relative_heading_to_opponent
        self.desired_heading_publisher.publish(newheading)
        self.manual_control_publisher.publish(cmd)

    def orbit(self):
        # Implement orbiting behavior around the opponent
        cmd = ManualControl()
        cmd.y = 100  # Neutral
        newheading = Int16()
        newheading.data = self.relative_heading_to_opponent
        self.desired_heading_publisher.publish(newheading)
        self.manual_control_publisher.publish(cmd)
        if self.depth_difference is not None:
            newdepth = Altitude
            newdepth.local = self.depth_difference
            self.desired_depth_publisher.publish(newdepth)
        else:
            newdepth = Altitude
            newdepth.local = 0.5
            self.desired_depth_publisher.publish(newdepth)

    def turn_on_lights(self, movement):
        """turns on auv lights"""
        movement.channels[8] = 2000
        movement.channels[9] = 2000    
        self.move_publisher.publish(movement)

    def turn_off_lights(self, movement):
        """turns off auv lights"""
        movement.channels[8] = 1000
        movement.channels[9] = 1000
        self.move_publisher.publish(movement)
    def flash(self):
        for i in range (3):
            self.turn_on_lights(self.movement)
            time.sleep(0.2)
            self.turn_off_lights(self.movement)
            time.sleep(0.2)
        self.turn_off_lights(self.movement)
        self.get_logger().info("Flashing light!")

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            self.update_state()
            self.perform_action()


def main(args=None):
    rclpy.init(args=args)
    node = AUVController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()





