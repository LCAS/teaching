#!/usr/bin/env python
import sys
import rospy
import cv2
import numpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):

    cv2.namedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
    #self.image_sub = rospy.Subscriber("/camera/rgb/image_color",Image,self.callback)
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
      print e

    bgr_thresh = cv2.inRange(cv_image, numpy.array((200, 230, 230)), numpy.array((255, 255, 255)))

    hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    hsv_thresh = cv2.inRange(hsv_img, numpy.array((90, 150, 0)), numpy.array((180, 255, 255)))

    print numpy.mean(hsv_img[:,:,0])
    print numpy.mean(hsv_img[:,:,1])
    print numpy.mean(hsv_img[:,:,2])

    bgr_contours, hierachy = cv2.findContours( bgr_thresh.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    hsv_contours, hierachy = cv2.findContours( hsv_thresh.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
    
    for c in hsv_contours:
        a = cv2.contourArea(c)
        if a > 100.0:
            #m = cv2.moments(c,True)
            #print m
            cv2.drawContours(hsv_thresh, c, -1, (128,0,0))
    print '===='    
    #print contours0
    #cv2.drawContours(cv_image, contours0, -1, (255,0,0))
    cv2.imshow("Image window", hsv_thresh)
    cv2.waitKey(3)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
    #while not rospy.is_shutdown():
        #cv2.waitKey(3)
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)