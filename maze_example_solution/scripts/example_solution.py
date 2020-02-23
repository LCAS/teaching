#!/usr/bin/env python
import rospy
import cv2 
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class example_maze_navigator:

    def __init__(self):
        cv2.startWindowThread()
        self.bridge = CvBridge()

        # subscribe to color image
        self.img_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.detect_color_blobs)

        # cmd_vel publisher 
        self.vel_pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=1)

    def detect_color_blobs(self, msg):
        cv2.namedWindow("raw image", 1)
        cv2.namedWindow("blue mask", 1)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            print e
        
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        hsv_blue_mask = cv2.inRange(hsv_img,
                                 np.array((100, 40, 70)),
                                 np.array((130, 255, 255)))
        




        cv2.imshow("raw image", cv_image)
        cv2.imshow("blue mask", hsv_blue_mask)
        cv2.waitKey(1)

    def move_forward(self):
        rospy.loginfo("-> moving forward")
        distance = 0.5 # metres
        speed = 0.2 # m/s
        rate = 10 # hz

        t = Twist()
        t.linear.x = speed

        r = rospy.Rate(rate) #hz

        steps = int(distance / speed * rate)
        for _ in range(steps):
            self.vel_pub.publish(t)
            r.sleep()

    def move_left(self):
        rospy.loginfo("-> turning left")
        distance = np.pi / 4 # radians
        speed = -np.pi / 4 # r/s
        rate = 10 # hz

        t = Twist()
        t.angular.z = speed

        r = rospy.Rate(rate) #hz

        steps = int(distance / np.abs(speed) * rate)

        for _ in range(steps):
            self.vel_pub.publish(t)
            r.sleep()

    def move_right(self):
        rospy.loginfo("-> turning right")
        distance = np.pi / 4 # radians
        speed = np.pi / 4 # r/s
        rate = 10 # hz

        t = Twist()
        t.angular.z = speed

        r = rospy.Rate(rate) #hz

        steps = int(distance / speed * rate)
        for _ in range(steps):
            self.vel_pub.publish(t)
            r.sleep()

    def navigate(self):

        while not rospy.is_shutdown():
            pass


navigator = example_maze_navigator()

rospy.init_node("example_maze_navigator")

## test primitive actions
navigator.move_forward()
navigator.move_right()
navigator.move_left()

## navigate the maze
navigator.navigate()

cv2.destroyAllWindows()
