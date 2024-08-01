#!/usr/bin/env python3

#~/ardupilot/Tools/autotest/sim_vehicle.py --vehicle=ArduSub --aircraft="bwsibot" -L RATBeach --out=udp:YOUR_COMPUTER_IP:14550
#ros2 launch /home/kenayosh/auvc_ws/src/AUV-Group-Github/launch/example.yaml

#cd ~/auvc_ws
#colcon build --symlink-install
#source ~/auvc_ws/install/setup.zsh

#ros2 topic list
#ros2 topic type /your/topic
#ro2 topic echo /your/topic :)))))
#ros2  interface show your_msg_library/msg/YourMessageType
#ros2 topic pub bluerov2/desired_depth mavros_msgs/msg/Altitude "{relative: 0.8}" 
#ros2 topic pub bluerov2/desired_heading std_msgs/msg/Int16 "{data: 3}" 

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from mavros_msgs.msg import ManualControl, Altitude
from std_msgs.msg import Int16

import numpy as np
import math
import matplotlib.pyplot as plt


class PIDHeadingNode(Node):
    def __init__(self):
        super().__init__('heading_node')
        #subscribers/publishers for necessary topics
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

        self.desired_heading_subscriber = self.create_subscription(
            Int16,
            'PID/desired_heading',
            self.desired_heading_callback,
            10
        )
        self.heading_derivative_subscriber = self.create_subscription(
            Imu,
            'bluerov2/imu',
            self.heading_derivative_callback,
            10
        )
        """
        PID CONSTANTS
        """
        self.kp = 2
        self.kd = 1
        self.min_output = -30.0
        self.max_output = 30.0
        self.previous_error = 0.0
        """"""

        self.get_logger().info('starting publisher node')
        #self.pid_yaw = PIDController(0.5, 0.1, 0.05, 1.0, -50, 50)
        """
        tracking constants
        """
        self.heading = None
        self.desired_heading = None
        self.heading_derivative = 0.0      
        self.array = np.array([])


    def compute(self, error):
        """
        computes and logs the correction power based on the angle error and angular velocity in rad/s as derivative
        """
        #tracking
        self.array = np.append(self.array, [(error)])

        #p and d calcs and return output
        self.derivative = self.heading_derivative
        proportional = self.kp * error
        output = proportional + (self.kd * self.derivative)

        #more tracking
        self.get_logger().info(f'\n Kp: {proportional} Kd: {self.kd *self.derivative} CurrentHeading: {self.heading}')
        
        #updates/clamping output
        output = max(min(output, self.max_output), self.min_output)
        self.previous_error = error
        return output

    def heading_callback(self, msg):
        """logs and stores int16 heading from subscriber"""
        self.heading = msg.data
        if self.desired_heading != None:
            self.calc_publish_heading()
        #self.get_logger().info(f'Depth: {self.depth}, Timestamp: {self.timestamp}')


    def desired_heading_callback(self, msg):
        """logs and stores desired heading from publisher in int16 and converts degrees to rads"""
        self.desired_heading = msg.data
        self.desired_heading = self.desired_heading

        
    def heading_derivative_callback(self, msg):
        """logs and stores angular velocity in deg/s from imu"""
        self.heading_derivative = msg.angular_velocity.z * 180/math.pi
    def calc_publish_heading(self):
        """calculates angular error using mod function and receives/publishes correction to manual control"""
        #checks to make sure that a heading has been recieved
        if self.heading is not None:
            #error calc
            error1 = ((self.desired_heading - self.heading + 180) % 360 - 180)/1.8
           # error2 = math.abs(100*math.sin(math.pi/180*x))*math.sin(x*math.pi/180)/math.abs(math.sin(x*math.pi/180))
            heading_correction = self.compute(error1)

            #publishing movement
            movement = ManualControl()

            movement.r = heading_correction
            self.get_logger().info(f'\nCurrent Power: {heading_correction}/100\nHeading: {self.heading}')
            self.move_publisher.publish(movement)


def main(args=None):
    rclpy.init(args=args)
    move_node = PIDHeadingNode()
    try:
        rclpy.spin(move_node)
    except KeyboardInterrupt:
        print('\nKeyboardInterrupt received, shutting down...')
    finally:
        x = np.arange(0,len(move_node.array))

        plt.plot(x,move_node.array)
        plt.savefig("/home/kenayosh/auvc_ws/src/AUV-Group-Github/intro_to_ros/heading_err.png")
        
        move_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
if __name__ == '__main__':
    main()




"""
- node:
    pkg: "intro_to_ros"
    exec: "pid"
    name: "pid"
    namespace: ""
"""