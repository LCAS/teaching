#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Image
import cv2
import cv_bridge


class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image,
                                          self.image_callback)

    def image_callback(self, msg):
        cv2.namedWindow("window", 1)
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow("window", image)
        cv2.waitKey(3)

#cv2.startWindowThread()
rospy.init_node('follower')
follower = Follower()
rospy.spin()
cv2.destroyAllWindows()
