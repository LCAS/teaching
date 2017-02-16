#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    cv2.namedWindow("window", 1)
    cv2.namedWindow("mask", 1)
    self.image_sub = rospy.Subscriber('/turtlebot_1/camera/rgb/image_raw', 
                                      Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher('/turtlebot_1/cmd_vel',
                                       Twist, queue_size=1)
    self.twist = Twist()
  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = numpy.array([ 40, 100,  80])
    upper_yellow = numpy.array([ 80, 255, 250])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    
    h, w, d = image.shape

    M = cv2.moments(mask)
    if M['m00'] > 0:
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
      cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
      # BEGIN CONTROL
      err = cx - w/2
      self.twist.linear.x = 0.2
      self.twist.angular.z = -float(err) / 100
      self.cmd_vel_pub.publish(self.twist)
      # END CONTROL
    cv2.imshow("window", image)
    cv2.imshow("mask", mask)
    cv2.waitKey(1)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
