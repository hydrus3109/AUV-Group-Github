#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

import cv2


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__("image_subscriber")

        self.cvb = CvBridge()

        self.subscription = self.create_subscription(
            Image, "bluerov2/camera", self.image_callback, 10
        )


    def detect_lines(self, img, threshold1=200, threshold2=300, apertureSize=5, minLineLength=600, maxLineGap= 50):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, threshold1, threshold2, apertureSize=apertureSize)
        
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, minLineLength, maxLineGap)
        if lines is not None:
            lines = [line[0] for line in lines]  # Extract the actual coordinates of the lines
        else:
            lines = []
        return lines

    def get_slopes_intercepts(self, lines):
        slopes = []
        intercepts = []
        for line in lines:
            x1, y1, x2, y2 = line
            if x2 - x1 != 0:  # Avoid division by zero
                slope = (y2 - y1) / (x2 - x1)
                intercept = y1 - slope * x1
            else:
                slope = None  # Vertical line case
                intercept = None
            slopes.append(slope)
            intercepts.append(intercept)
        return slopes, intercepts

    def detect_lanes(self, lines, slope_threshold=0.05, intercept_threshold=1):
        slopes, intercepts = self.get_slopes_intercepts(lines)
        lanes = []
        used_lines = set()  # Keep track of lines that have been assigned to a lane

        for i in range(len(lines)):
            if i in used_lines:
                continue  # Skip lines that have already been assigned to a lane

            best_pair = None
            best_intercept_diff = float('inf')

            for j in range(i + 1, len(lines)):
                if j in used_lines:
                    continue  # Skip lines that have already been assigned to a lane

                if slopes[i] is not None and slopes[j] is not None:
                    slope_diff = abs(slopes[i] - slopes[j])
                    intercept_diff = abs(intercepts[i] - intercepts[j])

                    if slope_diff < slope_threshold and intercept_diff > intercept_threshold:
                        if intercept_diff < best_intercept_diff:
                            best_intercept_diff = intercept_diff
                            best_pair = j

            if best_pair is not None:
                lanes.append([lines[i], lines[best_pair]])
                used_lines.add(i)
                used_lines.add(best_pair)

        return lanes

    def get_lane_center(self,lanes, img_width):
        if not lanes:
            return None, None
        
        # Initialize the closest lane variables
        closest_distance = float('inf')
        closest_intercept = None
        closest_slope = None
        img_center_x = img_width // 2

        # Compute center of each lane and find the closest
        for lane in lanes:
            for line in lane:
                slopes, intercepts = self.get_slopes_intercepts([line])
                distance = abs(img_center_x - intercepts[0])
                
                if distance < closest_distance:
                    closest_distance = distance
                    closest_slope = slopes[0]
                    closest_intercept = intercepts[0]

        return closest_intercept, closest_slope

    def recommend_direction(self, center_intercept, slope, img_width):
        if center_intercept is None or slope is None:
            return "unknown"
        img_center_x = img_width // 2
        if center_intercept is not None:
            return center_intercept - img_center_x

    def image_callback(self, msg: Image):
        """
        Callback function for the image subscriber.
        It receives an image message and saves it.

        Args:
            msg (Image): The image message
        """
        # Convert Image message to OpenCV image
        image = self.cvb.imgmsg_to_cv2(msg)

        # Save the image
        cv2.imwrite("image.png", image)


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()

    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        image_subscriber.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()





"""
def draw_lines(img, lines, color=(0, 255, 0)):
    for line in lines:
        x1, y1, x2, y2 = line
        cv2.line(img, (x1, y1), (x2, y2), color, 2)
    return img
"""