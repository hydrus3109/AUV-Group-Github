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
from intro_to_ros.armdisarm import *
from intro_to_ros.mode import *


class Dance(Node):
    def __init__(self):
        super().__init__("dance") #Node naame
        """
        inits the publisher for override
        """
        self.publisher = self.create_publisher(
            OverrideRCIn, #Type of message that's boreadcasted
            "/mavros/rc/override", #Topic name
            10
        )


        self.get_logger().info("starting publisher node")
        self.chachaslide()
        #self.test()
    def test(self):
        movement = OverrideRCIn()
        movement.channels = [65535] * 18
        self.orbit_left(movement, 6)
        self.orbit_right(movement, 6)
        self.wonky_left(movement, 6)
        self.wonky_right(movement, 6)
    def chachaslide(self):
        """
        the chacha slide function for the robot dance
        """
        movement = OverrideRCIn()
        movement.channels = [65535] * 18
        time.sleep(8.5)
        #funky for 4 sec (ccw)
        #funky for 2.2 sec (cw)
        #wait for 1.2 sec
        #clap
        #wait for .8
        #clap
        #wait .5
        #clap
        #wait .5
        #clap
        #wait 2
        #clap
        # wait 0.8
        #clap
        #wait .5
        #clap
        #wait .5
        #clap
        #wait 4.5
        #left for 1.4 sec
        #back for 1.4 sec
        #wait 0.7
        #1 hop for 1.5
        #wait .4
        #right stomp for 1 sec
        #wait for 1 sec
        #left stomp for 1 sec
        #wait for 1 sec

        
        #funky 
        self.turn_off_lights(movement)
        self.wonky_right(movement, 4)
        #funky
        self.wonky_left(movement, 2.2)
        time.sleep(1.2)
        #clap
        self.turn_on_lights(movement)
        time.sleep(.2)
        self.turn_off_lights(movement)
        time.sleep(.6)
        #clap
        self.turn_on_lights(movement)
        time.sleep(.2)
        self.turn_off_lights(movement)
        time.sleep(.3)
        #clap
        self.turn_on_lights(movement)
        time.sleep(.2)
        self.turn_off_lights(movement)
        time.sleep(.3)
        #clap
        self.turn_on_lights(movement)
        time.sleep(.2)
        self.turn_off_lights(movement)
        time.sleep(2)
        ##clap
        self.turn_on_lights(movement)
        time.sleep(.2)
        self.turn_off_lights(movement)
        time.sleep(.6)
        #clap
        self.turn_on_lights(movement)
        time.sleep(.2)
        self.turn_off_lights(movement)
        time.sleep(.3)
        #clap
        self.turn_on_lights(movement)
        time.sleep(.2)
        self.turn_off_lights(movement)
        time.sleep(.3)
        #clap
        self.turn_on_lights(movement)
        time.sleep(.2)
        self.turn_off_lights(movement)
        time.sleep(4.5)
        #left 
        self.move_left(movement, 75, 1)
        #take it back
        self.move_right(movement, 75, 1)
        time.sleep(.7)
        #hop
        self.move_up(movement, 100, .75)
        self.move_down(movement,100,.75)
        time.sleep(.4)
        #right foot stomp
        self.lean_right(movement, 100, 1)
        time.sleep(1)
        #left foot stomp
        self.lean_left(movement, 100, 1)
        time.sleep(1)
        #cha cha
        self.orbit_left(movement, 3.5)
        #turn it up
        self.turn_cw(movement, 100, 2.1)
        #to the left
        self.move_left(movement, 75, 2.6)
        #take it back
        self.move_backward(movement, 75, 1.5)
        #hop
        self.move_up(movement, 100, .5)
        self.move_down(movement,100,.5)
        time.sleep(1.4)  
        #right foot stomp
        self.lean_right(movement, 100, .7)
        time.sleep(.8)
        #left foot stomp
        self.lean_left(movement, 100, .6)
        time.sleep(1.6)
        #cha cha
        self.orbit_left(movement, 2)
        self.orbit_right(movement, 2)
        time.sleep(1.8)
        #to the right
        self.move_right(movement, 75, 1.6)
        #to the left
        self.move_left(movement, 75, 1.9)
        #backward
        self.move_backward(movement, 75, 1.5)
        #two hops
        self.move_up(movement, 100, .25)
        self.move_down(movement, 100, .25) 
        time.sleep(1.7) 
        self.move_up(movement, 100, .3)
        self.move_down(movement,100,.3)
        '''  
        #right foot two stomps
        self.lean_right(movement, 100, 1)
        time.sleep(1)
        self.lean_right(movement, 100, 1)
        #left foot two stomps
        self.lean_left(movement, 100, 1)
        time.sleep(1)
        self.lean_left(movement, 100, 1)
        #slide left
        self.move_left(movement, 100, 1)
        #slide right
        self.move_right(movement, 100, 1)
        #crisscross
        self.turn_ccw(movement, 100, 1)
        self.turn_cw(movement, 100, 1)
        self.turn_ccw(movement, 100, 1)
        self.turn_cw(movement, 100, 1)
        #chacha
        self.turn_ccw(movement, 100, 1)
        #lets go to work
        self.move_down(movement, 100, 1)
        self.move_up(movement, 100, 1)
        '''
        return 
        
    def reset(self, movement):
        """completely stops auv and resets movement"""
        movement.channels = [65535] * 18
        self.publisher.publish(movement)
        
    def move_forward(self, movement, speed, time):
        """moves auv forward given a speed between 0 and 100%, for a time in seconds"""
        movement.channels[4] = 1500 + 5*speed  # Forward
        self.publisher.publish(movement)
        sleep(time)
        movement.channels[4] = 1500

    def move_backward(self, movement, speed, time):
        """moves auv backward given a speed between 0 and 100%, for a time in seconds"""
        movement.channels[4] = 1500 - 5*speed  # Backward
        self.publisher.publish(movement)
        sleep(time)
        movement.channels[4] = 1500

    def move_left(self, movement, speed, time):
        """moves auv left given a speed between 0 and 100%, for a time in seconds"""
        movement.channels[5] = 1500 - 5*speed  # Forward
        self.publisher.publish(movement)
        sleep(time)
        movement.channels[5] = 1500
    def move_right(self, movement, speed, time):
        """moves auv right given a speed between 0 and 100%, for a time in seconds"""
        movement.channels[5] = 1500 + 5*speed  # Forward
        self.publisher.publish(movement)
        sleep(time)
        movement.channels[5] = 1500


    def move_up(self, movement, speed, time):
        """moves auv up given a speed between 0 and 100%, for a time in seconds"""
        movement.channels[2] = 1500 + 5*speed  # up
        self.publisher.publish(movement)
        sleep(time)
        movement.channels[2] = 1500
        
    def move_down(self, movement, speed, time):
        """moves auv down given a speed between 0 and 100%, for a time in seconds"""
        movement.channels[2] = 1500 - 5*speed  # down
        self.publisher.publish(movement)
        sleep(time)
        movement.channels[2] = 1500

    def turn_cw(self, movement, speed, time):
        """moves auv clockwise given a speed between 0 and 100%, for a time in seconds"""
        movement.channels[3] = 1500 + 5*speed  # Turn clockwise
        self.publisher.publish(movement)
        sleep(time)
        movement.channels[3] = 1500

    def turn_ccw(self, movement, speed, time):
        """moves auv counterclockwise given a speed between 0 and 100%, for a time in seconds"""
        movement.channels[3] = 1500 - 5*speed # Turn counterclockwise
        self.publisher.publish(movement)
        sleep(time)
        movement.channels[3] = 1500

    def turn_on_lights(self, movement):
        """turns on auv lights"""
        movement.channels[8] = 2000
        movement.channels[9] = 2000
        self.publisher.publish(movement)

    def turn_off_lights(self, movement):
        """turns off auv lights"""
        movement.channels[8] = 1500
        movement.channels[9] = 1500
        self.publisher.publish(movement)
    
    def lean_right(self, movement, speed, time):
        """leans auv to right given a speed between 0 and 100%, for a time in seconds"""
        movement.channels[0] = 1500 + 5*speed # Turn counterclockwise
        self.publisher.publish(movement)
        sleep(time)
        movement.channels[0] = 1500

    def lean_left(self, movement, speed, time):
        """leans auv to left given a speed between 0 and 100%, for a time in seconds"""
        movement.channels[0] = 1500 - 5*speed # Turn counterclockwise
        self.publisher.publish(movement)
        sleep(time)
        movement.channels[0] = 1500
    def orbit_left(self, movement, time):
        movement.channels[5] = 1000 
        movement.channels[3] = 1800
        self.publisher.publish(movement)
        sleep(time)
        movement.channels[5] = 1500
        movement.channels[3] = 1500
    def orbit_right(self, movement, time):
        movement.channels[5] = 2000  
        movement.channels[3] = 1200
        self.publisher.publish(movement)
        sleep(time)
        movement.channels[5] = 1500
        movement.channels[3] = 1500
    def wonky_right(self, movement, time):
        movement.channels[5] = 2000  
        movement.channels[3] = 2000
        self.publisher.publish(movement)
        sleep(time)
        movement.channels[5] = 1500
        movement.channels[3] = 1500
    def wonky_left(self, movement, time):
        movement.channels[5] = 1000  
        movement.channels[3] = 1000
        self.publisher.publish(movement)
        sleep(time)
        movement.channels[5] = 1500
        movement.channels[3] = 1500

def main(args=None):
    rclpy.init(args=args)           # starts the ROS2 Python3 client
    arm_node = ROVArmer()           # creates an instance of the RovArmDisarm class
    mode_node = ModeNode()  #mode set node
    arm_node.send_command(True)
    rclpy.spin_until_future_complete(arm_node, arm_node.arm_future)
    mode_node.set_mode()
    rclpy.spin_until_future_complete(mode_node, mode_node.future)
    
    try:
        move_node = Dance()             #Initializes moveNode
        rclpy.spin(move_node)            # keeps node running until there is an exception
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        arm_node.send_command(False) #Disarm the Rov
        arm_node.destroy_node()         # destroys node at the end of the program's lifespan
        move_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()        # closes the ROS2 Python3 client if it is still active

if __name__ == '__main__':
    main()