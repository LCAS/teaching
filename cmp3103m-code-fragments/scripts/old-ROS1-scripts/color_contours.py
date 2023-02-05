#!/usr/bin/env python

import rospy
import cv2
import numpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",
                                          Image, self.callback)

    def callback(self, data):
        cv2.namedWindow("Image window", 1)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e

        # using the BGR colour space, create a mask for everything
        # that is in a certain range
        bgr_thresh = cv2.inRange(cv_image,
                                 numpy.array((200, 230, 230)),
                                 numpy.array((255, 255, 255)))

        # It often is better to use another colour space, that is
        # less sensitive to illumination (brightness) changes.
        # The HSV colour space is often a good choice. 
        # So, we first change the colour space here...
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # ... and now let's create a binary (mask) image, looking for 
        # any hue (range: 0-255), but for something brightly
        # colours (high saturation: > 150)
        hsv_thresh = cv2.inRange(hsv_img,
                                 numpy.array((0, 150, 50)),
                                 numpy.array((255, 255, 255)))

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
        _, hsv_contours, hierachy = cv2.findContours(
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
                cv2.drawContours(cv_image, c, -1, (255, 0, 0), 3)
        print('====')
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(1)


image_converter()
rospy.init_node('image_converter', anonymous=True)
rospy.spin()
cv2.destroyAllWindows()
