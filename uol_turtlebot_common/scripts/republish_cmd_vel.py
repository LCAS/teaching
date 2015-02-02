#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from math import pi

class CmdVelRepublisher():
	"A class to republish cmd_vel information"

	def __init__(self):
		rospy.init_node('cmd_vel_republisher')
		self.x_lim = rospy.get_param("~x_lim", 0.55)
		self.z_lim = rospy.get_param("~z_lim", pi)
		self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist) 
		rospy.Subscriber("/cmd_vel", Twist, self.callback) 
		rospy.logdebug(rospy.get_name() + " setting up")

	def callback(self,msg): 
		out = Twist()
		rospy.logdebug(rospy.get_name() + ": I heard %s" % msg)
		out.linear.x = msg.linear.x if msg.linear.x <= self.x_lim else self.x_lim
		out.angular.z = msg.angular.z if msg.angular.z <= self.z_lim else self.z_lim
		self.pub.publish(out)

if __name__ == '__main__':
    republisher = CmdVelRepublisher()
    rospy.spin()
