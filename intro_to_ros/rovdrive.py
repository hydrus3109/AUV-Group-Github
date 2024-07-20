#!/usr/bin/env python3

#~/ardupilot/Tools/autotest/sim_vehicle.py --vehicle=ArduSub --aircraft="bwsibot" -L RATBeach --out=udp:YOUR_COMPUTER_IP:14550
#ros2 launch mavros apm.launch fcu_url:=udp://192.168.2.2:14550@14555 gcs_url:=udp://:14550@YOUR_COMPUTER_IP:14550 tgt_system:=1 tgt_component:=1 system_id:=255 component_id:=240

#cd ~/auvc_ws
#colcon build --packages-select intro_to_ros --symlink-install
#source ~/auvc_ws/install/setup.zsh

#ros2 topic list
#ros2 topic type /your/topic
#ro2 topic echo /your/topic :)))))
#ros2  interface show your_msg_library/msg/YourMessageType

import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool
from mavros_msgs.msg import OverrideRCIn, State
from mavros_msgs.srv import SetMode
from time import sleep

from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

class RovArmDisarmNode(Node):
    def __init__(self):
        super().__init__('arm_disarm')
        
        #Create subscriber to Arm/Disarm
        self.arm_state_subcription = self.create_subscription(
                            State,
                            "/mavros/state",
                            self.arm_sub_callback,
                            QoSProfile(
                            history=QoSHistoryPolicy.KEEP_LAST,
                            depth=10,
                            reliability=QoSReliabilityPolicy.BEST_EFFORT,
                            durability=QoSDurabilityPolicy.VOLATILE)
        )
        
        sleep(2)
        
        #Initalize Client to arm+disarm
        self.arm_client = self.create_client(CommandBool,               #Create a client with type matching service node..
                                             '/mavros/cmd/arming')      #and with name matching service node
        
        #Connect to Service
        while not self.arm_client.wait_for_service(timeout_sec=1.0):    #Every second, check if there is a service matching what we set above
            self.get_logger().info('/mavros/cmd/arming service not available, waiting again...')
        

    def send_arm_command(self, arm_bool: bool):
        """ Sends a command to arm or disarm the ROV """
        arm_request = CommandBool.Request()                             #Initializes a request of type CommandBool 
        arm_request.value = arm_bool                                    # True to arm, False to disarm
        
        self.arm_future = self.arm_client.call_async(arm_request)       #Make a service request (Future) and asynchronously get the result.              
        self.arm_future.add_done_callback(self.arm_response_callback)   #Add a callback to be executed when the task is done.
    
    def arm_response_callback(self, future):                        #When arm_request is done, this gets called
            try:
                response = future.result()                                  
                if (response.success == True):  #If arm_request future went through well
                    self.get_logger().info(f"Armed: {not self.arm_state}")
                else:
                    self.get_logger().error('Arming/Disarming failed with response: %r' % response.result)
            except Exception as e:
                self.get_logger().error('Service call failed %r' % (e,))
                
    def arm_sub_callback(self, msg):
        self.arm_state = msg.armed


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
        self.publisher_timer = self.create_timer(
            3.0, self.run_node #Every second, runs the run_node() method
        )
        self.get_logger().info("starting publisher node") #Log in the terminal "starting publisher node"

    def run_node(self):
        # Example of a neutral state setup
        movement = OverrideRCIn()
        movement.channels = [65535] * 18  # Setting all to middle value which typically means no action
        self.move_forward(movement)
        self.publisher.publish(movement)
        self.get_logger().info(f"OverrideRCIn: {movement.channels}")

    def move_forward(self, movement):
        movement.channels[4] = 2000  # Forward

    def move_backward(self, movement):
        movement.channels[4] = 1000  # Backward

    def move_up(self, movement):
        movement.channels[2] = 2000  # Up

    def move_down(self, movement):
        movement.channels[2] = 1000  # Down

    def turn_cw(self, movement):
        movement.channels[3] = 2000  # Turn clockwise

    def turn_ccw(self, movement):
        movement.channels[3] = 1000  # Turn counterclockwise

    def strafe_right(self, movement):
        movement.channels[5] = 2000  # Strafe right

    def strafe_left(self, movement):
        movement.channels[5] = 1000  # Strafe left

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