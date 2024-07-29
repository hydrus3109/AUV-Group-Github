from enum import Enum, auto
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, Range
from mavros_msgs.msg import ManualControl, Altitude
from std_msgs.msg import Int16
import numpy as np

class State(Enum):
    SCANNING = auto()
    MOVING = auto()
    ORBITING = auto()
    FLASHING = auto()

class GameState:
    def __init__(self, heading, depth, distance_to_opponent, relative_heading_to_opponent):
        self.heading = heading
        self.depth = depth
        self.distance_to_opponent = distance_to_opponent
        self.relative_heading_to_opponent = relative_heading_to_opponent

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
        
        self.current_state = State.SCANNING
        self.current_heading = None
        self.current_depth = None
        self.distance_to_opponent = None
        self.relative_heading_to_opponent = None

    def camera_callback  (self, msg):
        # Process image to detect the opponent and update distance and relative heading
        pass
        self.update_state()

    def depth_callback(self, msg):
        self.current_depth = msg.relative

    def heading_callback(self, msg):
        self.heading = msg.data

    def update_state(self):
        # Update the current state based on sensor data
        self.current_game_state = GameState(
            heading=self.current_heading,
            depth=self.current_depth,
            distance_to_opponent=self.distance_to_opponent,
            relative_heading_to_opponent=self.relative_heading_to_opponent
        )
        self.transition_state()

    def transition_state(self):
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
        # Implement scanning behavior, e.g., rotating in place to find the opponent
        cmd = ManualControl()
        cmd.x = 1500  # Neutral
        cmd.y = 1500  # Neutral
        cmd.z = 1500  # Neutral
        cmd.r = 1600  # Rotate right
        self.manual_control_publisher.publish(cmd)

    def move(self):
        # Implement moving behavior towards the opponent
        cmd = ManualControl()
        cmd.x = 1600  # Move forward
        cmd.y = 1500  # Neutral
        cmd.z = 1500  # Neutral
        cmd.r = 1500 + int((self.relative_heading_to_opponent / 180.0) * 500)  # Adjust heading
        self.manual_control_publisher.publish(cmd)

    def orbit(self):
        # Implement orbiting behavior around the opponent
        cmd = ManualControl()
        cmd.x = 1500  # Neutral
        cmd.y = 1600  # Move right
        cmd.z = 1500  # Neutral
        cmd.r = 1500  # Maintain current heading
        self.manual_control_publisher.publish(cmd)

    def flash(self):
        # Implement flashing behavior
        # This is a placeholder, actual implementation will depend on the hardware and how the flashing is triggered
        self.get_logger().info("Flashing light!")

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            self.update_state()
            self.perform_action()

def main(args=None):
    rclpy.init(args=args)
    controller = AUVController()
    controller.run()
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




