#!/usr/bin/env python

import rospy
from cv2 import namedWindow, cvtColor, imshow
from cv2 import destroyAllWindows, startWindowThread
from cv2 import COLOR_BGR2GRAY
from numpy import mean
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class image_converter:

    def __init__(self):

        namedWindow("Image window", 1)
        self.bridge = CvBridge()
        startWindowThread()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",
                                          Image, self.callback)
        #self.image_sub = rospy.Subscriber("/turtlebot_1/camera/rgb/image_raw",Image,self.callback)

    def callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        gray_img = cvtColor(cv_image, COLOR_BGR2GRAY)
        print mean(gray_img)

        imshow("Image window", gray_img)

ic = image_converter()
rospy.init_node('image_converter')
rospy.spin()

destroyAllWindows()
