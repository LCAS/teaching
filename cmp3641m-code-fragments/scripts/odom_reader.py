#!/usr/bin/env python

import rospy
from pprint import pformat
from tf_conversions import transformations
from math import pi


from nav_msgs.msg import Odometry


class odom_reader:

    def __init__(self):
        self.image_sub = rospy.Subscriber("/turtlebot_1/odom",
                                          Odometry, self.callback)

    """
    convert an orientation given in quaternions to an actual
    angle in degrees for a 2D robot
    """
    def odom_orientation(self, q):
        y, p, r = transformations.euler_from_quaternion([q.w, q.x, q.y, q.z])
        return y * 180 / pi

    def callback(self, data):
        print "odom pose: \n" + pformat(data.pose.pose)
        angle = self.odom_orientation(data.pose.pose.orientation)
        print "angle = %f" % angle

ic = odom_reader()
rospy.init_node('odom_reader')
rospy.spin()
