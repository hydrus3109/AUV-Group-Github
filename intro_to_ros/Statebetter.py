from enum import Enum, auto
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, Range
from mavros_msgs.msg import ManualControl, Altitude, OverrideRCIn
from std_msgs.msg import Int16, Float32, Bool
import numpy as np
import time

class State(Enum):
    PREGAME = auto()
    SCANNING = auto()
    CHASE = auto()
    FLASHING = auto()

class StateBetter(Node):
    MIN_DEPTH = 0.3                             #Minimum depth we can dive
    MAX_DEPTH = 0.8                             #Maximum depth we can dive
    MIN_DISTANCE_TO_OPPONENT = 2                #Distance to opponent before we can shoot
    START_HEADING = 60
    def __init__(self):
        super().__init__('state_better')
        self.IMG_heading_subscriber     = self.create_subscription(Int16   ,'img/desired_heading', self.IMG_heading_callback, 10)
        self.AT_distance_subscriber     = self.create_subscription(Float32  ,"img/distance", self.AT_distance_callback, 10) #distance to the AprilTag
        self.targetted_subscriber       = self.create_subscription(Bool     ,'img/targetted', self.targetted_callback, 10) #if both AprilTag and YOLO fails
        self.more_lanes_subscriber      = self.create_subscription(Int16    ,'ld/more_lanes', self.more_lanes_callback, 10) #check if there are more lanes to get ready to turn
        self.lateral_offset_subscriber  = self.create_subscription(Float32  ,'ld/lateral_offset', self.lateral_offset_callback, 10) #how far are we to the lane
        self.Robot_heading_subscriber   = self.create_subscription(Int16    ,'bluerov2/heading', self.robot_heading_callback, 10)

        self.direct_manual_publisher        = self.create_publisher(ManualControl,'bluerov2/manual_control',10) #just for x and y
        self.direct_lights_publisher        = self.create_publisher(OverrideRCIn,'bluerov2/override_rc',10) #just for lights
        self.PID_desired_depth_publisher    = self.create_publisher(Altitude, 'PID/desired_depth', 10) #just for lights
        self.PID_desired_heading_publisher  = self.create_publisher(Int16,'PID/desired_heading', 10)

        self.target_found = False
        self.start_heading = None
        self.IMG_heading = None
        self.distance_to_opponent = None
        self.lane_found = True
        self.current_depth = None
        self.lights_on = 10

    def IMG_heading_callback(self, msg):
        self.IMG_heading = msg.data   
    
    def AT_distance_callback(self, msg):
        if(self.target_found):
            self.distance_to_opponent = msg.data
        else:
            self.distance_to_opponent = None

    def lateral_offset_callback(self, msg):
        self.lateral_offset = msg.data          #offset from the lanes

    def more_lanes_callback(self, msg):        
        self.lane_found = msg.data      

    def robot_heading_callback(self, msg):
        if self.start_heading is None:
            self.start_heading = msg.data
        self.current_heading = msg.data

    def targetted_callback(self,msg):   
        if(msg.data):
            self.target_found = True
        else:
            self.target_found = False

    def pregame(self):                          #Assuming we can place our robot not head-on with the opponent
        depth_msg = Altitude()
        depth_msg.relative = self.MAX_DEPTH
        self.PID_desired_depth_publisher.publish(depth_msg)

        msg = Int16()
        msg.data = self.START_HEADING                      #Angle of the lanes (measured)
        self.PID_desired_heading_publisher.publish(msg)

        movement = ManualControl()
        movement.x = 100                        # Move forward with the lanes

        if self.lateral_offset is not None:
            movement.y = min(self.lateral_offset, 20)
            self.get_logger().info(f'\nCurrent Power for lateral offset: {movement.y}')
            self.direct_manual_publisher.publish(movement)
    
    def scan(self):
        """scanning behaviour for finding other robot"""

        depth_msg = Altitude()
        depth_msg.relative = (self.MAX_DEPTH + self.MIN_DEPTH)/2
        self.PID_desired_depth_publisher.publish(depth_msg)

        #moves forward slowly
        movement = ManualControl()
        movement.x = 20
        self.direct_manual_publisher.publish(movement)

        desired_depth = Altitude()
        desired_depth.relative = self.MAX_DEPTH   # We want to go to the maximum possible depth in our range
        self.PID_desired_depth_publisher.publish(desired_depth)

        newheading = Int16()
        self.get_logger().info(f'Start Heading: {self.start_heading}')

        # newheading.data = self.start_heading + 20 #Rotate 20 degrees to the left and right to maximize scanning range
        # self.PID_desired_heading_publisher.publish(newheading)
        # time.sleep(1) #wait for robot to reach this heading

        # newheading.data = self.start_heading - 20 #Rotate the other way
        # self.PID_desired_heading_publisher.publish(newheading)
        # time.sleep(1) #wait for robot to reach this heading

        # newheading.data = self.start_heading    # Come back to starting position
        # self.PID_desired_heading_publisher.publish(newheading)
        # time.sleep(1)

        if self.lateral_offset is not None:     #make sure we stick to the lane
            movement.y = min(self.lateral_offset, 20)
            self.get_logger().info(f'\nCurrent Power for lateral offset: {movement.y}')
            self.direct_manual_publisher.publish(movement)

        self.direct_manual_publisher.publish(movement)
        #time.sleep(1)

    def chase(self):
        '''Implement moving behavior towards the opponent'''
        movement = ManualControl()
        movement.x = 50                              # Move forward

        newheading = Int16()                    # sets our PID heading to the angle we detected the AprilTag/model
        newheading.data = self.IMG_heading
        self.PID_desired_heading_publisher.publish(newheading)
        self.direct_manual_publisher.publish(movement)

        msg = Altitude()
        msg.relative = (self.MAX_DEPTH + self.MIN_DEPTH)/2
        self.PID_desired_depth_publisher.publish(msg)

        # Moving behavior towards the opponent
        movement = ManualControl()
        movement.x = 30  # Move forward

        #set heading according to what is found from camera subscriber
        newheading = Int16()
        if(self.IMG_heading is None):
            self.get_logger().info("[ERR:COOKED] IDK WHATS HAPPENING BUT TARGET HEADING IS NONE IN THE CHASE FUNCTIONNNN")
            return
        newheading.data = self.IMG_heading
        self.PID_desired_heading_publisher.publish(newheading)
        self.direct_manual_publisher.publish(movement)


    def flash(self, movement):
        """Flashes lights at the AUV"""
        #Maintains middle depth
        RCmovement = OverrideRCIn()
        msg = Altitude()
        msg.relative = (self.MAX_DEPTH + self.MIN_DEPTH)/2
        self.PID_desired_depth_publisher.publish(msg)

        #sets up a timer, lights_on is a counter, each 10 frames the light either turns on or turns off
        self.lights_on += 1
        if(self.lights_on > 10):
            RCmovement.channels[8] = 2000
            RCmovement.channels[9] = 2000    
            self.direct_lights_publisher.publish(RCmovement)
        elif(self.lights_on > 20):
            self.lights_on = 0
            RCmovement.channels[8] = 10000
            RCmovement.channels[9] = 1000
            self.direct_lights_publisher.publish(RCmovement)

        #make sure that we stop when we're flashing the lights
        ManualMovement = ManualControl()
        ManualMovement.x = 0
        self.direct_manual_publisher.publish(ManualMovement)
        

    def perform_action(self):
        if self.current_state == State.PREGAME:
            self.pregame()
        elif self.current_state == State.SCANNING:
            self.scan()
        elif self.current_state == State.CHASE:
            self.chase()
        elif self.current_state == State.FLASHING:
            self.flash()
    
    def update_state(self):
        """Handles state based transitions"""
        # Update the current state based on sensor data
        if self.current_state == State.PREGAME:
            if not self.lane_found:
                #Turns the AUV around by setting desired heading
                msg = Int16()
                msg.data = 240
                self.PID_desired_heading_publisher.publish(msg)

                self.get_logger().info("---ENTERING SCANNING STAGE---")
                self.current_state = State.SCANNING

        # Helps transition from scanning state to chasing state
        if self.current_state == State.SCANNING:
            if self.distance_to_opponent is not None and not self.target_found:
                self.get_logger().info("---FOUND AUV, ENTERING CHASING STAGE---")
                self.current_state = State.CHASE

        # Helps transition from chasing state back to scanning state if auv lost or to flashing state if auv distance is close
        elif self.current_state == State.CHASE:
            if self.distance_to_opponent is None:
                self.get_logger().info("---AUV LOST, ENTERING BACK TO SCANNING STAGE---")
                self.current_state = State.SCANNING
            elif self.distance_to_opponent < self.MIN_DISTANCE_TO_OPPONENT:
                self.get_logger().info("---AUV CLOSE, FLASHING LIGHTS---")
                self.current_state = State.FLASHING

        # Transitions from flashing state back to chasing state if AUV distance is too big or AUV lost
        elif self.current_state == State.FLASHING:
            if self.distance_to_opponent is None and self.distance_to_opponent > self.MIN_DISTANCE_TO_OPPONENT:
                self.get_logger().info("---AUV ESCAPED, ENTERING BACK TO CHASING STAGE---")
                self.current_state = State.CHASE
    
    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            self.update_state()
            self.perform_action()

def main(args=None):
    rclpy.init(args=args)
    node = StateBetter()

    try:
        node.run()
        # rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()