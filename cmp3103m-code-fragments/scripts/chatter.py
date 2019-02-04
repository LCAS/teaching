# -*- coding: utf-8 -*-
"""
Created on Wed Jan 28 15:40:31 2015

@author: lcas
"""

import rospy
from std_msgs.msg import String


class Chatter:

    def __init__(self):
        rospy.init_node('chatter')
        self.publisher = rospy.Publisher('/msgs', String, queue_size=1)

    def run(self):
        while not rospy.is_shutdown():
            s = String()
            s.data = 'Hi!'
            self.publisher.publish(s)
            rospy.sleep(1)

c = Chatter()
c.run()
