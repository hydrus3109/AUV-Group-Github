#!/usr/bin/env python3

#~/ardupilot/Tools/autotest/sim_vehicle.py --vehicle=ArduSub --aircraft="bwsibot" -L RATBeach --out=udp:YOUR_COMPUTER_IP:14550
#ros2 launch mavros apm.launch fcu_url:=udp://192.168.2.2:14550@14555 gcs_url:=udp://:14550@YOUR_COMPUTER_IP:14550 tgt_system:=1 tgt_component:=1 system_id:=255 component_id:=240

#cd ~/auvc_ws
#colcon build --packages-select intro_to_ros --symlink-install
#source ~/auvc_ws/install/setup.zsh

#ros2 launch /home/kenayosh/auvc_ws/src/AUV-Group-Github/launch/_.yaml
#ros2 topic list
#ros2 topic type /your/topic
#ro2 topic echo /your/topic :)))))
#ros2  interface show your_msg_library/msg/YourMessageType

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from cv_bridge import CvBridge
import numpy as np
from std_msgs.msg import Int16, Float32, Bool
import cv2
import matplotlib.pyplot as plt
from dt_apriltags import Detector
from time import sleep
from ultralytics import YOLO

FOV_HOR = 80 
FOV_VER = 64 


class YOLOSubscriber(Node):
    def __init__(self):
        super().__init__("bluerov")
        self.subscriber = self.create_subscription(Image,"bluerov2/camera",self.image_callback,10)
        self.get_logger().info("starting camera subscriber node")
        self.heading_subscriber = self.create_subscription(Int16,'bluerov2/heading',self.heading_callback,10)
        self.get_logger().info("starting heading SUB node")
        self.IMG_heading_publisher = self.create_publisher(Int16,"img/desired_heading",10)
        self.targetted_publisher = self.create_publisher(Bool,"img/targetted",10)
        self.get_logger().info("starting heading publsiher node")
        self.get_logger().info("starting heading PUB node")
        self.distance_publisher = self.create_publisher(
            Float32,
            "img/distance",
            10
        )
        self.get_logger().info("starting distance PUB node")
        
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
        self.target_msg = Bool()
        self.AT_heading_message = Int16()
        self.AT_distance_message = Float32()
        self.targetting_fails = 0   
        self.model = YOLO("intro_to_ros/best_ncnn_model")
        self.yolo_done = True
        self.at_done = True
        
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
    
    #publish all apriltag stuff
    def send_april_tags(self, img, tags):
        for tag in tags:
            self.x_angle = self.calculate_rel_horizontal_angle(img, tag)
            self.y_angle = self.calculate_rel_verticle_angle(img, tag)
            self.z_distance = self.calculate_distance(img, tag)
            #self.get_logger().info(f"X Angle: {self.x_angle}, Y Angle: {self.y_angle}, Z Distance: {self.z_distance}")
            
            self.AT_distance_message.data = self.z_distance*1.0
            self.distance_publisher.publish(self.AT_distance_message)
            
            if (self.heading != None) and (self.x_angle != None):
                self.AT_heading_message.data = self.heading + int(self.x_angle)
                # self.get_logger().info(f"relative heading: {int(self.x_angle)}")
                self.IMG_heading_publisher.publish(self.AT_heading_message)
          
    def calculate_publish_yolo(self, image):
        self.yolo_done = False
        imgwidth = np.shape(image)[1]
        imgheight = np.shape(image)[0]
        image = image[int(imgheight*0.20):imgheight, 0:imgwidth]
        model = YOLO("intro_to_ros/best_ncnn_model")
        results = model(image)
        if results is None:
            self.get_logger().info(f"Nothing found in anything")
            self.yolo_done = True
            self.get_logger().info("No Yolo results")

            self.target_msg = Bool()
            self.target_msg.data = False
            self.targetted_publisher.publish(self.target_msg)
            self.AT_heading_message.data = None
            self.IMG_heading_publisher.publish(self.AT_heading_message)

            return
        self.get_logger().info("Yolo Found")
        boxes = results[0].boxes
        conf = boxes.conf.numpy().squeeze()
        confidx = -1
        bestconf = 0
        for i in range(len(conf)):
            if conf[i] > bestconf and conf[i] > 0.6:
                bestconf = conf[i]
                confidx = i
        if confidx != -1:
            coords= boxes.xyxy.numpy().squeeze()[confidx]
            xcenter =(coords[0]+coords[2])/2
        
        if (self.heading != None) and (xcenter != None):
            desired_heading = int(FOV_HOR*(xcenter-image.shape[1]/2)/image.shape[1])
            self.YOLO_heading_message = Int16()
            self.YOLO_heading_message.data = int(desired_heading + self.heading)
            self.get_logger().info(f"Publishing Yolo Heading: {self.YOLO_heading_message.data}")
            self.IMG_heading_publisher.publish(self.YOLO_heading_message)
        else:
            self.get_logger().info(f"Heading or xcenter is none in Yolo")
        self.yolo_done = True
        
    def image_callback(self,msg):
        if(not self.at_done):
            return
        self.at_done = False
        
        img = self.bridge.imgmsg_to_cv2(msg)
        
        # plt.imsave("/home/kenayosh/auvc_ws/src/AUV-Group-Github/intro_to_ros/images/Camera_feed.png", img)
        
        imgwidth = np.shape(img)[1]
        imgheight = np.shape(img)[0]
        # img = img[int(imgheight*0.20):imgheight, 0:imgwidth]
        if img.any()!=None:
            frame_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).copy() #and convert to gray
            tags = self.at_detector.detect(frame_gray, estimate_tag_pose=True, camera_params=[1000,1000,img.shape[1]/2,img.shape[0]/2], tag_size=0.1)
            
            # self.get_logger().info("Checking AT")
            # self.get_logger().info(f"tags that were found: {tags}")
            self.get_logger().info("Starting to find Tags")
            if len(tags) > 0:   #April tag found
                # self.get_logger().info("FOUND AT")
                self.get_logger().info("Found April Tags")
                self.send_april_tags(img, tags)
                self.target_msg = Bool()
                self.target_msg.data = True
                self.targetted_publisher.publish(self.target_msg)
                sleep(0.3)
            elif self.yolo_done == True: #No april tag AND yolo is done
                self.get_logger().info("Starting to find YOLO")
                self.calculate_publish_yolo(img)
            else: #no april tag but yolo sin't done
                self.get_logger().info(f"Nothing found in anything")
                self.target_msg = Bool()
                self.target_msg.data = False
                self.targetted_publisher.publish(self.target_msg)
                self.AT_heading_message.data = None
                self.IMG_heading_publisher.publish(self.AT_heading_message)
                # self.get_logger().("Did not find AT")
                
                # model = YOLO("best.pt")  # load a pretrained model (recommended for training)
                # image = self.bridge.imgmsg_to_cv2(msg)
                
                # imgwidth = np.shape(image)[1]
                # imgheight = np.shape(image)[0]
                # image = image[int(imgheight*0.35):imgheight, 0:imgwidth]
                
                # results = model(image)  # return a list of Results objects
                # if(len(results) != 0):    
                #     boxes = results[0].boxes
                #     x1 = boxes.xyxy
                #     self.get_logger().info(x1)
                #     annotated_frame = results[0].plot()
                #     plt.imshow(annotated_frame)
                    
                #     self.target_msg = Bool()
                #     self.target_msg.data = True
                #     self.targetted_publisher.publish(self.target_msg)
                #     sleep(0.5)
                # else:
                #     #nothing found ur cooked buddy
                #     self.target_msg = Bool()
                #     self.target_msg.data = False
                #     self.targetted_publisher.publish(self.target_msg)
        self.at_done = True
        

def main(args=None):
    rclpy.init(args=args)
    node = YOLOSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()