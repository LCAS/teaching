#!/usr/bin/env python

import rospy
import cv2
import numpy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

    def __init__(self):

        cv2.startWindowThread()
        self.bridge = CvBridge()
        # self.image_sub = rospy.Subscriber("/camera/image_raw",
        #                                   Image, self.callback)
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",
                                          Image, self.callback)
        self.err_pub = rospy.Publisher('error', Float64, queue_size=1)

    def callback(self, data):
        cv2.namedWindow("Image window", 1)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e

        h, w, d = cv_image.shape

        bgr_thresh = cv2.inRange(cv_image,
                                 numpy.array((200, 230, 230)),
                                 numpy.array((255, 255, 255)))

        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        hsv_thresh = cv2.inRange(hsv_img,
                                 numpy.array((0, 150, 50)),
                                 numpy.array((255, 255, 255)))

        M = cv2.moments(hsv_thresh)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            print('cx: %f, cy: %f' %(cx, cy))
            cv2.circle(cv_image, (cx, cy), 20, (255, 0, 0), -1)
            err = cx - w/2
            self.err_pub.publish(err)


        cv2.imshow("Image window", cv_image)
        cv2.waitKey(1)


image_converter()
rospy.init_node('image_converter', anonymous=True)
rospy.spin()
cv2.destroyAllWindows()
