#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from math import pi
from kobuki_msgs.msg import BumperEvent
from threading import Thread

class CmdVelRepublisher():
    "A class to republish cmd_vel information"

    def __init__(self):
        rospy.init_node('cmd_vel_republisher')
        self.disabled = False
        self.thread = None
        self.x_lim = rospy.get_param("~x_lim", 0.55)
        self.z_lim = rospy.get_param("~z_lim", pi)
        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist)
        rospy.Subscriber('/mobile_base/events/bumper', BumperEvent,self.bumper_cb)
        rospy.Subscriber("/cmd_vel", Twist, self.callback)
        rospy.logdebug(rospy.get_name() + " setting up")

    def callback(self,msg):
        if not self.is_disabled():
            out = Twist()
            rospy.logdebug(rospy.get_name() + ": I heard %s" % msg)
            out.linear.x = msg.linear.x if msg.linear.x <= self.x_lim else self.x_lim
            out.angular.z = msg.angular.z if msg.angular.z <= self.z_lim else self.z_lim
            self.pub.publish(out)
        else:
            rospy.logwarn("cmd vel republisher disabled")

    def stop(self):
        while self.is_disabled() and not rospy.is_shutdown():
            out = Twist()
            out.linear.x = 0.
            out.angular.z = 0.
            self.pub.publish(out)
            rospy.sleep(0.1)

    def bumper_cb(self, msg):
        if msg.state:
            self.disabled = msg.state
            self.thread = Thread(target=self.stop)

    def is_disabled(self):
        return self.disabled

if __name__ == '__main__':
    republisher = CmdVelRepublisher()
    rospy.spin()
