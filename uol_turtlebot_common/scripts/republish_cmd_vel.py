#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from math import pi
from kobuki_msgs.msg import BumperEvent, Led
from threading import Thread
from std_srvs.srv import Empty, EmptyResponse

class CmdVelRepublisher():
    "A class to republish cmd_vel information"

    def __init__(self):
        rospy.init_node('cmd_vel_republisher')
        self.disabled = False
        self.stop_thread = None
        self.led_thread = None
        self.x_lim = rospy.get_param("~x_lim", 0.55)
        self.z_lim = rospy.get_param("~z_lim", pi)
        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        self.pub_led1 = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=10)
        self.pub_led2 = rospy.Publisher('/mobile_base/commands/led2', Led, queue_size=10)
        rospy.Service("/reset_bumper_stop", Empty, self.enable)
        rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.bumper_cb)
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

    def leds(self):
        while self.is_disabled() and not rospy.is_shutdown():
            self.pub_led1.publish(Led(value=2))
            self.pub_led2.publish(Led(value=3))
            rospy.sleep(0.5)
            self.pub_led1.publish(Led(value=3))
            self.pub_led2.publish(Led(value=2))
            rospy.sleep(0.5)

        self.pub_led1.publish(Led(value=0))
        self.pub_led2.publish(Led(value=0))

    def bumper_cb(self, msg):
        if not self.is_disabled() and msg.state:
            self.disabled = msg.state
            self.stop_thread = Thread(target=self.stop)
            self.stop_thread.start()
            self.led_thread = Thread(target=self.leds)
            self.led_thread.start()

    def enable(self, req):
        self.disabled = False
        if self.stop_thread:
            self.stop_thread.join()
        if self.led_thread:
            self.led_thread.join()
        return EmptyResponse()

    def is_disabled(self):
        return self.disabled

if __name__ == '__main__':
    republisher = CmdVelRepublisher()
    rospy.spin()
