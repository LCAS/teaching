#!/usr/bin/env python
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_publisher:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_pub = rospy.Publisher('/image', Image, latch=True)

  def publish(self,filename):
    cv_image = cv2.imread(filename)

    self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

def main(args):
  ic = image_publisher()
  rospy.init_node('image_converter', anonymous=True)
  try:
    ic.publish('blofeld.jpg')
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"

if __name__ == '__main__':
    main(sys.argv)