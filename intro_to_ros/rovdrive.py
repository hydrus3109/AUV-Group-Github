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
from mavros_msgs.msg import OverrideRCIn, State, Imu
from mavros_msgs.srv import SetMode
from time import sleep
from std_msgs.msg import Float32
import tf_transformations
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
        self.imu_subscription = self.create_subscription(Imu, '/mavros/imu/data', self.imu_callback, 10)
        self.depth_subscription = self.create_subscription(Float32, '/mavros/depth', self.depth_callback, 10)
        
        self.current_yaw = 0.0
        self.current_depth = 0.0

        # Initialize PID controllers
        self.pid_yaw = PIDController(0.5, 0.1, 0.05, 5.0, -100, 100)
        self.pid_depth = PIDController(0.5, 0.1, 0.05, 5.0, -100, 100)
        self.publisher_timer = self.create_timer(
            3.0, self.run_node #Every second, runs the run_node() method
        )
        self.get_logger().info("starting publisher node") #Log in the terminal "starting publisher node"
    def imu_callback(self, msg):
        # Extract yaw from quaternion
        orientation_q = msg.orientation
        euler = tf_transformations.euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w])
        self.current_yaw = euler[2]  # Yaw is the third element

    def pressure_to_depth(self, pressure):
        """calculates depth in meters given presssure in Pascals"""
        rho = 1029  # density of sea water in kg/m^3 
        g = 9.81  # acceleration due to gravity in m/s^2
        pressure_at_sea_level = 1013.25  # average sea level pressure in hPa
        return (pressure - pressure_at_sea_level) * 100 / (rho * g)
    def pressure_callback(self, msg):
        self.current_depth = self.pressure_to_depth(msg.data)

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


    def Move_lateral(self):
        """moves auv in given direction"""
        # Example of a neutral state setup
        movement = OverrideRCIn()
        movement.channels = [65535] * 18  # Setting all to middle value which typically means no action
        self.move_forward(movement)
        self.publisher.publish(movement)
        self.get_logger().info(f"OverrideRCIn: {movement.channels}")

    def move_forward(self, movement, speed):
        """moves auv forward given a speed between 0 and 100"""
        movement.channels[4] = 1500 + 5*speed  # Forward

    def move_backward(self, movement, speed):
        """moves auv backward given a speed between 0 and 100"""
        movement.channels[4] = 1500 - 5*speed  # Backward

    def move_left(self, movement, speed):
        """moves auv left given a speed between 0 and 100"""
        movement.channels[5] = 1500 - 5*speed  # Forward
    def move_right(self, movement, speed):
        """moves auv right given a speed between 0 and 100"""
        movement.channels[5] = 1500 + 5*speed  # Forward


    def move_up(self, movement, speed):
        """moves auv up given a speed between 0 and 100"""
        movement.channels[2] = 1500 + 5*speed  # up
        
    def move_down(self, movement, speed):
        """moves auv down given a speed between 0 and 100"""
        movement.channels[2] = 1500 - 5*speed  # down

    def turn_cw(self, movement):
        movement.channels[3] = 2000  # Turn clockwise

    def turn_ccw(self, movement):
        movement.channels[3] = 1000  # Turn counterclockwise


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