#!/usr/bin/env python

# An example of TurtleBot 3 subscribe to camera topic and mask colours
# Written for humble

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2
import numpy as np

class ColourMask(Node):
    def __init__(self):
        super().__init__('colour_mask')
        self.pub_video_hsv = self.create_publisher(Image, 'video/hsv', 10)
        self.pub_video_mask = self.create_publisher(Image, 'video/mask', 10)
        self.create_subscription(Image, '/limo/depth_camera_link/image_raw', self.camera_callback, 10)

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
        cv2.namedWindow("Image window", 1)

    def camera_callback(self, data):
        #self.get_logger().info("camera_callback")

        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data, "bgr8")

        # Convert image to HSV
        current_frame_hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)
        # Create mask for range of colours (HSV low values, HSV high values)
        # Onine colour picker - https://redketchup.io/color-picker
        #current_frame_mask = cv2.inRange(current_frame_hsv,(70, 40, 20), (150, 100, 255))
        current_frame_mask = cv2.inRange(current_frame_hsv,(0, 220, 50), (10, 255, 255))

        # Convert OpenCV image to ROS Image message and publish topic
        self.pub_video_hsv.publish(self.br.cv2_to_imgmsg(current_frame_hsv))
        self.pub_video_mask.publish(self.br.cv2_to_imgmsg(current_frame_mask))
        #self.get_logger().info('Publishing video frame')
        cv2.imshow("Image window", current_frame_hsv)
        cv2.waitKey(1)

def main(args=None):
    print('Starting colour_mask.py.')

    rclpy.init(args=args)

    colour_mask = ColourMask()

    rclpy.spin(colour_mask)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    colour_mask.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()