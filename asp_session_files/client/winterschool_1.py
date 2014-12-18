#! /usr/bin/env python

import roslib; roslib.load_manifest('rosoclingo_examples')
from rosoclingo.msg import *
import rospy
import actionlib

client = actionlib.ActionClient('ROSoClingo',ROSoClingoAction)

def send_goal(content):
 goal = ROSoClingoGoal("moveto(" + str(content) + ")")
 print "send: " + "bring(" + str(content) + ")"
 return client.send_goal(goal)

def example():
 handlers = []
 client.wait_for_server() 

 goal = "6"
 handlers.append(send_goal(goal))
 rospy.sleep(10.0)

 goal = "3"
 handlers.append(send_goal(goal))
 rospy.sleep(40.0)

 goal = "2"
 handlers.append(send_goal(goal))
 rospy.sleep(30.0)

 goal = "4"
 handlers.append(send_goal(goal))
 rospy.sleep(20.0)

if __name__ == '__main__':
 try:
  rospy.init_node('winterschool_manual')
  example()
  rospy.spin()
 except rospy.ROSInterruptException:
  print "program interrupted before completion"
