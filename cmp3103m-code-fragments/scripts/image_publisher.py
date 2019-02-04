#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class image_publisher:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher('/image', Image, latch=True)

    def publish(self, filename):
        cv_image = cv2.imread(filename)

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

ic = image_publisher()
rospy.init_node('image_converter', anonymous=True)
ic.publish('../blofeld.jpg')
rospy.spin()
