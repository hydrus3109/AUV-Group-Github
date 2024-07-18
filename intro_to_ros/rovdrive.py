#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Vector3

class PIDController:
    def __init__(self, kp, ki, kd, max_integral, min_output, max_output):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_integral = max_integral
        self.min_output = min_output
        self.max_output = max_output
        self.integral = 0.0
        self.previous_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.previous_error = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        self.integral = max(min(self.integral, self.max_integral), -self.max_integral)  # Anti-windup

        derivative = (error - self.previous_error) / dt
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        output = max(min(output, self.max_output), self.min_output)  # Clamp to max/min output

        self.previous_error = error
        return output

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        self.velocity_publisher = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', qos_profile)
        self.distance_subscriber = self.create_subscription(Odometry, '/mavros/local_position/odom', self.distance_callback, qos_profile)
        self.target_distance = 3.0  # Target distance in meters
        self.pid = PIDController(kp=0.5, ki=0.1, kd=0.05, max_integral=0.2, min_output=-2.0, max_output=2.0)
        self.current_distance = 0.0
        self.timer = self.create_timer(0.1, self.control_loop)  # Control loop at 10 Hz

    def distance_callback(self, msg):
        self.current_distance = msg.pose.pose.position.x
        self.get_logger().info(str(self.current_distance))

    def control_loop(self):
        error = self.target_distance - self.current_distance
        correction = self.pid.compute(error, 0.1)  
        velocity_command = TwistStamped()
        velocity_command.twist.linear.x = correction  # Set the velocity based on PID output
        velocity_command.twist.angular.z = 0.0  # Ensure no rotational movement
        self.velocity_publisher.publish(velocity_command)
        self.get_logger().info(f'Velocity Command: {correction:.2f} m/s')

def main(args=None):
    rclpy.init(args=args)
    control_node = ControlNode()
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

