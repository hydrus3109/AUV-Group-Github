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
from mavros_msgs.msg import OverrideRCIn
from time import sleep
import math
from mavros_msgs.msg import Altitude
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


class PIDNode(Node):
    def __init__(self):
        super().__init__("move_node") #Node name
        
        self.move_publisher = self.create_publisher(                    #Initialize the publisher
            OverrideRCIn, #Type of message that's boreadcasted
            "bluerov2/override_rc", #Topic name
            10
        )
        self.depth_subscriber = self.create_subscription(
            Altitude, 
            "bluerov2/depth",
            self.depth_callback,
            10
        )
        
        self.movement = OverrideRCIn()
        self.movement.channels = [65535] * 18                                #Initialize the movement channel Type (OverrideRCIn)
        self.get_logger().info("starting publisher node")
        self.pid_yaw = PIDController(0.5, 0.1, 0.05, 5.0, -100, 100)
        self.pid_depth = PIDController(0.5, 0.1, 0.05, 5.0, -100, 100)
    def temperature_callback(self, msg):
        self.depth = msg.relative
        self.get_logger().info(f"Depth: {self.depth}")

    def stop(self, time):
        self.movement.channels = [1500] * 18
        self.move_publisher.publish(self.movement)
        sleep(time)
    
    def Move_rotate(self, desired):
        yaw_correction = self.pid_yaw.compute(desired - self.current_yaw, 0.1)
        movement = OverrideRCIn()
        movement.channels = [65535] * 18
        movement.channels[3] = int(1500 + yaw_correction)
        self.publisher.publish(movement)
    
    
    def Move_updown(self, desired):
        depth_correction = self.pid_depth.compute(desired- self.depth, 0.1)
        movement = OverrideRCIn()
        movement.channels = [65535] * 18
        movement.channels[2] = int(1500 + depth_correction)
        self.publisher.publish(movement)
    
        #self.test()
    
        
    def destroy_node(self):
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)           # starts the ROS2 Python3 client
    move_node = PIDNode()    
    try:            #Initializes moveNode
        rclpy.spin(move_node)            # keeps node running until there is an exception
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:      
        move_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()        # closes the ROS2 Python3 client if it is still active

if __name__ == '__main__':
    main()