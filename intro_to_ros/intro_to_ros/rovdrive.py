#!/usr/bin/env python3

#~/ardupilot/Tools/autotest/sim_vehicle.py --vehicle=ArduSub --aircraft="bwsibot" -L RATBeach --out=udp:YOUR_COMPUTER_IP:14550
#ros2 launch mavros apm.launch fcu_url:=udp://192.168.2.2:14550@14555 gcs_url:=udp://:14550@YOUR_COMPUTER_IP:14550 tgt_system:=1 tgt_component:=1 system_id:=1 component_id:=240

#cd ~/auvc_ws
#colcon build --packages-select intro_to_ros --symlink-install
#source ~/auvc_ws/install/setup.zsh

#ros2 topic list
#ros2 topic type /your/topic
#ro2 topic echo /your/topic :)))))
#ros2  interface show your_msg_library/msg/YourMessageType

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from mavros_msgs.srv import CommandBool
from mavros_msgs.msg import OverrideRCIn, State
from mavros_msgs.srv import SetMode
from time import sleep
from std_msgs.msg import Float32
import time
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy


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
    
class RovArmDisarmNode(Node):
    def __init__(self):
        super().__init__('arm_disarm')
        
        #Initalize Client to arm+disarm
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/mavros/cmd/arming service not available, waiting again...')

    def send_arm_command(self, arm: bool):
        """ Sends a command to arm or disarm the ROV based on the bool given"""
        req = CommandBool.Request()
        req.value = arm  # True to arm, False to disarm
        self.arm_future = self.arm_client.call_async(req)
        self.arm_future.add_done_callback(self.arm_response_callback)

    def arm_response_callback(self, future):
        """
        gets response from arming and confirms whether or not robot is armed
        """
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Arming/Disarming successful' if future.result().success else 'Arming/Disarming unsuccessful')
            else:
                self.get_logger().error('Arming/Disarming failed with response: %r' % response.result)
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,))


class ModeNode(Node):
    def __init__(self):
        super().__init__('set_mode_client')
        self.client = self.create_client(SetMode, '/mavros/set_mode')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /mavros/set_mode not available, waiting again...')
        self.request = SetMode.Request()

    def set_mode(self):
        self.request.custom_mode = 'MANUAL'
        self.future = self.client.call_async(self.request)
        self.future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info('Mode set to MANUAL successfully')
            else:
                self.get_logger().info('Failed to set mode to MANUAL')
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,))


class MoveNode(Node):
    def __init__(self):
        super().__init__("motor_publisher") #Node name
        self.publisher = self.create_publisher(
            OverrideRCIn, #Type of message that's boreadcasted
            "/mavros/rc/override", #Topic name
            10
        )
        #self.imu_subscription = self.create_subscription(Imu, '/mavros/imu/data', self.imu_callback, 10)
       # self.depth_subscription = self.create_subscription(Float32, '/mavros/depth', self.depth_callback, 10)
        
        self.current_yaw = 0.0
        self.current_depth = 0.0

        # Initialize PID controllers
        self.pid_yaw = PIDController(0.5, 0.1, 0.05, 5.0, -100, 100)
        self.pid_depth = PIDController(0.5, 0.1, 0.05, 5.0, -100, 100)
        self.time = self.create_timer(0.5, self.Move_lateral)
        self.get_logger().info("starting publisher node") #Log in the terminal "starting publisher node"
        #self.Move_lateral()
    def imu_callback(self, msg):
        # Extract yaw from quaternion
        orientation_q = msg.orientation
        self.current_yaw = orientation_q.w # Yaw is the third element

    def pressure_to_depth(self, pressure):
        """calculates depth in meters given presssure in Pascals"""
        rho = 1029  # density of sea water in kg/m^3 
        g = 9.81  # acceleration due to gravity in m/s^2
        pressure_at_sea_level = 1013.25  # average sea level pressure in hPa
        return (pressure - pressure_at_sea_level) * 100 / (rho * g)
    def pressure_callback(self, msg):
        self.current_depth = self.pressure_to_depth(msg.data)
    """
    def Move_rotate(self, desired):
        yaw_correction = self.pid_yaw.compute(desired - self.current_yaw, 0.1)
        movement = OverrideRCIn()
        movement.channels = [65535] * 18
        movement.channels[3] = int(1500 + yaw_correction)
        self.publisher.publish(movement)
    """
    """
    def Move_updown(self, desired):
        depth_correction = self.pid_depth.compute(desired- self.current_depth, 0.1)
        movement = OverrideRCIn()
        movement.channels = [65535] * 18
        movement.channels[2] = int(1500 + depth_correction)
        self.publisher.publish(movement)
    """

    def Move_lateral(self):
        """moves auv in given direction"""
        # Example of a neutral state setup
        movement = OverrideRCIn()
        movement.channels = [65535] * 18  # Setting all to middle value which typically means no action
        self.turn_ccw(movement, 50, 3)
        self.get_logger().info(f"OverrideRCIn: {movement.channels}")
        
    def chachaslide(self):
        movement = OverrideRCIn()
        movement.channels = [65535] * 18 
        self.move_left(movement, 100, 3)
        self.move_backward()
        return 

    def reset(self, movement):
        movement.channels = [65535] * 18
        self.publisher.publish(movement)
        
    def move_forward(self, movement, speed, time):
        """moves auv forward given a speed between 0 and 100"""
        movement.channels[4] = 1500 + 5*speed  # Forward
        self.publisher.publish(movement)
        sleep(time)
        movement.channels[4] = 1500

    def move_backward(self, movement, speed, time):
        """moves auv backward given a speed between 0 and 100"""
        movement.channels[4] = 1500 - 5*speed  # Backward
        self.publisher.publish(movement)
        sleep(time)
        movement.channels[4] = 1500

    def move_left(self, movement, speed, time):
        """moves auv left given a speed between 0 and 100"""
        movement.channels[5] = 1500 - 5*speed  # Forward
        self.publisher.publish(movement)
        sleep(time)
        movement.channels[5] = 1500
    def move_right(self, movement, speed, time):
        """moves auv right given a speed between 0 and 100"""
        movement.channels[5] = 1500 + 5*speed  # Forward
        self.publisher.publish(movement)
        sleep(time)
        movement.channels[5] = 1500


    def move_up(self, movement, speed, time):
        """moves auv up given a speed between 0 and 100"""
        movement.channels[2] = 1500 + 5*speed  # up
        self.publisher.publish(movement)
        sleep(time)
        movement.channels[2] = 1500
        
    def move_down(self, movement, speed, time):
        """moves auv down given a speed between 0 and 100"""
        movement.channels[2] = 1500 - 5*speed  # down
        self.publisher.publish(movement)
        sleep(time)
        movement.channels[2] = 1500

    def turn_cw(self, movement, speed, time):
        movement.channels[3] = 1500 + 5*speed  # Turn clockwise
        self.publisher.publish(movement)
        sleep(time)
        movement.channels[3] = 1500

    def turn_ccw(self, movement, speed, time):
        movement.channels[3] = 1500 - 5*speed # Turn counterclockwise
        self.publisher.publish(movement)
        sleep(time)
        movement.channels[3] = 1500


def main(args=None):
    rclpy.init(args=args)           # starts the ROS2 Python3 client
    arm_node = RovArmDisarmNode()           # creates an instance of the RovArmDisarm class
    mode_node = ModeNode()
    arm_node.send_arm_command(True)
    rclpy.spin_until_future_complete(arm_node, arm_node.arm_future)
    mode_node.set_mode()
    rclpy.spin_until_future_complete(mode_node, mode_node.future)
    
    try:
        move_node = MoveNode()             #Initializes moveNode
        rclpy.spin(move_node)            # keeps node running until there is an exception
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        arm_node.send_arm_command(False) #Disarm the Rov
        arm_node.destroy_node()         # destroys node at the end of the program's lifespan
        move_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()        # closes the ROS2 Python3 client if it is still active

if __name__ == '__main__':
    main()