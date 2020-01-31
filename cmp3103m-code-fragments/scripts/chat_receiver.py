# -*- coding: utf-8 -*-
"""
Created on Wed Jan 28 15:40:31 2015

@author: lcas
"""

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class Receiver:

    def __init__(self):
        rospy.init_node('receiver')
        self.subscriber = rospy.Subscriber(
            '/msgs', String, callback=self.cb)

    def cb(self, msg):
        print(msg)

rec = Receiver()

rospy.spin()
