#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from cv_bridge import CvBridge
import numpy as np
from std_msgs.msg import Int16
import cv2
import matplotlib.pyplot as plt
from dt_apriltags import Detector

FOV_HOR = 80 
FOV_VER = 64 

class CameraSubscriber(Node):
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
        
        self.heading_publisher = self.create_subscription(
            Int16,
            "bluerov2/desired_heading",
            10
        )
        self.get_logger().info("starting heading publsiher node")
        
        self.heading = None
        self.x_angle = None
        self.y_angle = None
        self.z_distance = None
        
        
    def calculate_rel_horizontal_angle(img, tag):
        x = tag.center[0]
        return FOV_HOR*(x-img.shape[1]/2)/img.shape[1]
    
    def calculate_rel_verticle_angle(img, tag):
        y = tag.center[1]
        return FOV_VER*(y-img.shape[0]/2)/img.shape[0]

    def calculate_distance(img, tag):
        return np.linalg.norm(tag.pose_t)
        
    def heading_callback(self, msg):
        """logs and stores int16 heading from subscriber"""
        self.heading = msg.data   
        
    def image_callback(self, msg):
        bridge = CvBridge()
        img = bridge.cv2_to_imgmsg(msg, encoding="bgr8")
        
        at_detector = Detector(families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)

        tags = at_detector.detect(img, estimate_tag_pose=False, camera_params=None, tag_size=None)
        for tag in tags:
            self.x_angle = self.calculate_rel_horizontal_angle(tag)
            y_angle = self.calculate_rel_verticle_angle(tag)
            z_distance = self.calculate_distance(tag)
            self.logger(f"X Angle: {self.x_angle}, Y Angle: {y_angle}, Z Distance: {z_distance}")
            if (self.heading != None) and (self.x_angle != None):
                self.heading_publisher.publish(self.heading + self.x_angle)
            
            
            