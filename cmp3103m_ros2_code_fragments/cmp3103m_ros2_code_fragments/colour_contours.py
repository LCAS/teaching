#!/usr/bin/env python

# Written for humble
# cv2 image types - http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError # Package to convert between ROS and OpenCV Images
import cv2
import numpy as np

class ColourContours(Node):
    def __init__(self):
        super().__init__('colour_contours')
        self.create_subscription(Image, '/limo/depth_camera_link/image_raw', self.camera_callback, 1)

        self.br = CvBridge()

    def camera_callback(self, data):
        #self.get_logger().info("camera_callback")

        cv2.namedWindow("Image window", 1)
        try:
            cv_image = self.br.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # using the BGR colour space, create a mask for everything
        # that is in a certain range
        bgr_thresh = cv2.inRange(cv_image,
                                 np.array((200, 230, 230)),
                                 np.array((255, 255, 255)))

        # It often is better to use another colour space, that is
        # less sensitive to illumination (brightness) changes.
        # The HSV colour space is often a good choice. 
        # So, we first change the colour space here...
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # ... and now let's create a binary (mask) image, looking for 
        # any hue (range: 0-255), but for something brightly
        # colours (high saturation: > 150)
        hsv_thresh = cv2.inRange(hsv_img,
                                 np.array((0, 150, 50)),
                                 np.array((255, 255, 255)))

        # just for the fun of it, print the mean value 
        # of each HSV channel within the mask 
        print(cv2.mean(hsv_img[:, :, 0], mask = hsv_thresh)[0])
        print(cv2.mean(hsv_img[:, :, 1], mask = hsv_thresh)[0])
        print(cv2.mean(hsv_img[:, :, 2], mask = hsv_thresh)[0])

        # This is how we could find actual contours in
        # the BGR image, but we won't do this now.
        # _, bgr_contours, hierachy = cv2.findContours(
        #     bgr_thresh.copy(),
        #     cv2.RETR_TREE,
        #     cv2.CHAIN_APPROX_SIMPLE)

        # Instead find the contours in the mask generated from the
        # HSV image.
        hsv_contours, hierachy = cv2.findContours(
            hsv_thresh.copy(),
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE)
        
        # in hsv_contours we now have an array of individual
        # closed contours (basically a polgon around the 
        # blobs in the mask). Let's iterate over all those found 
        # contours.
        for c in hsv_contours:
            # This allows to compute the area (in pixels) of a contour
            a = cv2.contourArea(c)
            # and if the area is big enough, we draw the outline
            # of the contour (in blue)
            if a > 100.0:
                cv2.drawContours(cv_image, c, -1, (0, 0, 255), 10)
        print('====')

        cv_image_small = cv2.resize(cv_image, (0,0), fx=0.4, fy=0.4) # reduce image size
        cv2.imshow("Image window", cv_image_small)
        cv2.waitKey(1)

def main(args=None):
    print('Starting colour_contours.py.')

    rclpy.init(args=args)

    colour_contours = ColourContours()

    rclpy.spin(colour_contours)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    colour_contours.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()