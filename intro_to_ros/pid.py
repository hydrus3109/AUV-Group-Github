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


class MoveNode(Node):
    def __init__(self):
        super().__init__("move_node") #Node name
        
        self.move_publisher = self.create_publisher(                    #Initialize the publisher
            OverrideRCIn, #Type of message that's boreadcasted
            "bluerov2/override_rc", #Topic name
            10
        )
        
        self.movement = OverrideRCIn()
        self.movement.channels = [65535] * 18                                #Initialize the movement channel Type (OverrideRCIn)
        self.move_publisher.publish(self.movement)
        self.get_logger().info("Starting publisher node for movement") #Log in the terminal "starting publisher node"
        self.get_logger().info("starting publisher node")
        self.pid_yaw = PIDController(0.5, 0.1, 0.05, 5.0, -100, 100)
        self.pid_depth = PIDController(0.5, 0.1, 0.05, 5.0, -100, 100)
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
        depth_correction = self.pid_depth.compute(desired- self.current_depth, 0.1)
        movement = OverrideRCIn()
        movement.channels = [65535] * 18
        movement.channels[2] = int(1500 + depth_correction)
        self.publisher.publish(movement)
    
        #self.test()
    def test(self):
        movement = OverrideRCIn()
        movement.channels = [65535] * 18
        self.move_forward(movement, 30, 5)
        
        
    def reset(self, movement):
        """completely stops auv and resets movement"""
        movement.channels = [65535] * 18
        self.move_publisher.publish(movement)
        
    def move_forward(self, movement, speed, time):
        """moves auv forward given a speed between 0 and 30%, for a time in seconds"""
        for i in range(math.floor(time*10)):
            movement.channels[4] = 1500 + 5*speed  # Forward
            self.move_publisher.publish(movement)
            sleep(.1)
        movement.channels[4] = 1500

    def move_backward(self, movement, speed, time):
        """moves auv backward given a speed between 0 and 30%, for a time in seconds"""
        for i in range(math.floor(time*10)):
            movement.channels[4] = 1500 - 5*speed  # Backward
            self.move_publisher.publish(movement)
            sleep(0.1)
        movement.channels[4] = 1500

    def move_left(self, movement, speed, time):
        """moves auv left given a speed between 0 and 30%, for a time in seconds"""
        for i in range (math.floor(time*10)):
            movement.channels[5] = 1500 - 5*speed  # Forward
            self.move_publisher.publish(movement)
            sleep(.1)
        movement.channels[5] = 1500
    def move_right(self, movement, speed, time):
        """moves auv right given a speed between 0 and 30%, for a time in seconds"""
        for i in range (math.floor(time*10)):
            movement.channels[5] = 1500 + 5*speed  # Forward
            self.move_publisher.publish(movement)
            sleep(.1)
        movement.channels[5] = 1500


    def move_up(self, movement, speed, time):
        """moves auv up given a speed between 0 and 30%, for a time in seconds"""
        for i in range (math.floor(time*10)):
            movement.channels[2] = 1500 + 5*speed  # up
            self.move_publisher.publish(movement)
            sleep(.1)
        movement.channels[2] = 1500
        
    def move_down(self, movement, speed, time):
        """moves auv down given a speed between 0 and 30%, for a time in seconds"""
        for i in range (math.floor(time*10)):
            movement.channels[2] = 1500 - 5*speed  # down
            self.move_publisher.publish(movement)
            sleep(.1)
        movement.channels[2] = 1500

    def turn_cw(self, movement, speed, time):
        """moves auv clockwise given a speed between 0 and 30%, for a time in seconds"""
        for i in range (math.floor(time*10)):
            movement.channels[3] = 1500 + 5*speed  # Turn clockwise
            self.move_publisher.publish(movement)
            sleep(.1)
        movement.channels[3] = 1500

    def turn_ccw(self, movement, speed, time):
        """moves auv counterclockwise given a speed between 0 and 30%, for a time in seconds"""
        for i in range (math.floor(time*10)):
            movement.channels[3] = 1500 - 5*speed # Turn counterclockwise
            self.move_publisher.publish(movement)
            sleep(.1)
        movement.channels[3] = 1500

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
    
    def lean_right(self, movement, speed, time):
        """leans auv to right given a speed between 0 and 30%, for a time in seconds"""
        for i in range (math.floor(time*10)):
            movement.channels[1] = 1500 + 5*speed # Turn counterclockwise
            self.move_publisher.publish(movement)
            sleep(.1)
        movement.channels[1] = 1500

    def lean_left(self, movement, speed, time):
        """leans auv to left given a speed between 0 and 30%, for a time in seconds"""
        for i in range (math.floor(time*10)):
            movement.channels[1] = 1500 - 5*speed # Turn counterclockwise
            self.move_publisher.publish(movement)
            sleep(.1)
        movement.channels[1] = 1500
    def orbit_left(self, movement, time):
        for i in range (math.floor(time*10)):
            movement.channels[5] = 1000 
            movement.channels[3] = 1750
            self.move_publisher.publish(movement)
            sleep(.1)
        movement.channels[5] = 1500
        movement.channels[3] = 1500
    def orbit_right(self, movement, time):
        for i in range (math.floor(time*10)):
            movement.channels[5] = 2000  
            movement.channels[3] = 1250
            self.move_publisher.publish(movement)
            sleep(.1)
        movement.channels[5] = 1500
        movement.channels[3] = 1500
    def wonky_right(self, movement, time):
        for i in range (math.floor(time*10)):
            movement.channels[5] = 2000  
            movement.channels[3] = 1700
            self.move_publisher.publish(movement)
            sleep(.1)
        movement.channels[5] = 1500
        movement.channels[3] = 1500
    def wonky_left(self, movement, time):
        for i in range (math.floor(time*10)):
            movement.channels[5] = 1000  
            movement.channels[3] = 1200
            self.move_publisher.publish(movement)
            sleep(.1)
        movement.channels[5] = 1500
        movement.channels[3] = 1500
        
    def destroy_node(self):
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)           # starts the ROS2 Python3 client
    move_node = MoveNode()    
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