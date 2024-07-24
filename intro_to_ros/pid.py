import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from mavros_msgs.msg import ManualControl, Altitude


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
        self.error_accumulator = 0.0

    def reset(self):
        self.integral = 0.0
        self.previous_error = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        self.integral = max(min(self.integral, self.max_integral), -self.max_integral)

        derivative = (error - self.previous_error) / dt

        proportional = self.kp * error
        output = proportional + (self.ki * self.integral) + (self.kd * derivative)
        output = max(min(output, self.max_output), self.min_output)

        self.previous_error = error
        return output


class PIDNode(Node):
    def __init__(self):
        super().__init__('move_node')

        self.move_publisher = self.create_publisher(
            ManualControl,
            'bluerov2/manual_control',
            10
        )
        self.depth_subscriber = self.create_subscription(
            Altitude,
            'bluerov2/depth',
            self.depth_callback,
            10
        )

        self.desired_depth_subscriber = self.create_subscription(
            Altitude,
            'bluerov2/desired_depth',
            self.desired_depth_callback,
            10
        )

        self.get_logger().info('starting publisher node')
        #self.pid_yaw = PIDController(0.5, 0.1, 0.05, 1.0, -50, 50)
        self.pid_depth = PIDController(30, 5, 2, 10.0, -100.0, 100.0)
        self.depth = 0.0
        self.desired_depth = 5.0
        self.vert_timer = self.create_timer(0.1, self.calc_publish_vertical)

    def depth_callback(self, msg):
        self.depth = msg.relative
        #self.get_logger().info(f'Depth: {self.depth}')

    def desired_depth_callback(self, msg):
        #self.desired_depth = msg.relative
        print("hi") 
    def calc_publish_vertical(self):
        if self.depth is not None:
            depth_correction = self.pid_depth.compute(self.depth - self.desired_depth, 0.1)
            movement = ManualControl()
            movement.z = depth_correction
            self.get_logger().info(f'\nCurrent Speed: {depth_correction}\nDepth: {self.depth}')
            self.move_publisher.publish(movement)

    def move_vertical(self, desired):
        self.desired_depth = desired
        if not self.vert_timer.is_ready():
            self.vert_timer.reset()

    def stop_timer(self):
        self.vert_timer.cancel()
        self.get_logger().info('Timer stopped.')

def main(args=None):
    rclpy.init(args=args)
    move_node = PIDNode()
    try:
        rclpy.spin(move_node)
    except KeyboardInterrupt:
        print('\nKeyboardInterrupt received, shutting down...')
    finally:
        move_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
