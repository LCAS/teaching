#! /usr/bin/env python

import rospy
from rosoclingo.msg import ROSoClingoOut, ROSoClingoIn

topic = rospy.Publisher('/ROSoClingo/in', ROSoClingoIn,latch=True)

def rosoclingoout_callback(message):
 out_message = ROSoClingoIn()
 out_message.id = message.id
 out_message.value = "success"
# rospy.sleep(2.5)
 topic.publish(out_message)

rospy.init_node('interface_fulfill')
rospy.Subscriber("/ROSoClingo/out",ROSoClingoOut,rosoclingoout_callback)
rospy.spin()
