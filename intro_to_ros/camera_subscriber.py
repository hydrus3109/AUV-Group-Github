#!/usr/bin/env python

#usr/bin/python3 -m pip install dt_apriltags --break-system-packages

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from cv_bridge import CvBridge
import numpy as np
from std_msgs.msg import Int16, Float64
import cv2
import matplotlib.pyplot as plt
from dt_apriltags import Detector
import time

FOV_HOR = 80 
FOV_VER = 64 


class CameraSubscriber(Node):
    Done = True
    def __init__(self):
        super().__init__("bluerov")
        self.subscriber = self.create_subscription(
            Image,
            "bluerov2/camera",
            self.image_callback,
            10
        )
        self.get_logger().info("starting camera subscriber node")
        
        self.heading_subscriber = self.create_subscription(
            Int16,
            'bluerov2/heading',
            self.heading_callback,
            10
        )
        self.get_logger().info("starting heading subscriber node")
        
        self.heading_publisher = self.create_publisher(
            Int16,
            "bluerov2/desired_heading",
            10
        )
        self.get_logger().info("starting heading publsiher node")
        
        self.heading = None
        self.x_angle = None
        self.y_angle = None
        self.z_distance = None
        self.bridge = CvBridge()
        self.at_detector = Detector(families='tag36h11',
                            nthreads=3,
                            quad_decimate=1.0,
                            quad_sigma=0.0,
                            refine_edges=1,
                            decode_sharpening=0.25,
                            debug=0)
        
        
    def calculate_rel_horizontal_angle(self, img, tag):
        x = tag.center[0]
        return FOV_HOR*(x-img.shape[1]/2)/img.shape[1]
    
    def calculate_rel_verticle_angle(self, img, tag):
        y = tag.center[1]
        return FOV_VER*(y-img.shape[0]/2)/img.shape[0]

    def calculate_distance(self, img, tag):
        return np.linalg.norm(tag.pose_t)
        
    def heading_callback(self, msg):
        """logs and stores int16 heading from subscriber"""
        self.heading = msg.data   
        
    def image_callback(self, msg):
        self.get_logger().info("A")
        if not self.Done:
            return
        self.get_logger().info("B")
        self.Done = False
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if img.any()!=None:
            frame_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) #and convert to gray
            
            tags = self.at_detector.detect(frame_gray, estimate_tag_pose=True, camera_params=[1000,1000,img.shape[1]/2,img.shape[0]/2], tag_size=0.1)
            
            if len(tags) > 0:
                for tag in tags:
                    self.x_angle = self.calculate_rel_horizontal_angle(img, tag)
                    y_angle = self.calculate_rel_verticle_angle(img, tag)
                    z_distance = self.calculate_distance(img, tag)
                    self.get_logger().info(f"X Angle: {self.x_angle}, Y Angle: {y_angle}, Z Distance: {z_distance}")
                    if (self.heading != None) and (self.x_angle != None):
                        message = Int16()
                        message.data = self.heading + int(self.x_angle)
                        self.heading_publisher.publish(message)
            self.Done = True            
        
                    
            
            
            

def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()