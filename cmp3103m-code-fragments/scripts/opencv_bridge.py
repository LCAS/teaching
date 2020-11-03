#!/usr/bin/env python

import rospy
from cv2 import namedWindow, cvtColor, imshow
from cv2 import destroyAllWindows, startWindowThread
from cv2 import COLOR_BGR2GRAY, waitKey
from cv2 import blur, Canny
from numpy import mean
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class image_converter:

    def __init__(self):

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",
                                          Image, self.image_callback)
        # self.image_sub = rospy.Subscriber(
        #     "/camera/rgb/image_raw",
        #     Image, self.callback)

    def image_callback(self, data):
        namedWindow("Image window")
        namedWindow("blur")
        namedWindow("canny")
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        gray_img = cvtColor(cv_image, COLOR_BGR2GRAY)
        print mean(gray_img)
        img2 = blur(gray_img, (3, 3))
        imshow("blur", img2)
        img3 = Canny(gray_img, 10, 200)
        imshow("canny", img3)

        imshow("Image window", cv_image)
        waitKey(1)

# not needed in newer versions: 
# startWindowThread()
rospy.init_node('image_converter')
ic = image_converter()
rospy.spin()

# destroyAllWindows()
