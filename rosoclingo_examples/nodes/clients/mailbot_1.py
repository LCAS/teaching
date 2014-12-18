#! /usr/bin/env python

import roslib; roslib.load_manifest('rosoclingo_examples')
from rosoclingo.msg import *
import rospy
import actionlib

client = actionlib.ActionClient('ROSoClingo',ROSoClingoAction)

def send_goal(fr,to):
 goal = ROSoClingoGoal("bring(" + str(fr) + "," + str(to) + ")")
 print "send: " + "bring(" + str(fr) + "," + str(to) + ")"
 return client.send_goal(goal)

def example():
 handlers = []
 client.wait_for_server()
 handlers.append(send_goal(5,1))
 rospy.sleep(1.0)
 handlers.append(send_goal(1,5))
 rospy.sleep(1.0)
 handlers.append(send_goal(2,4))
 rospy.sleep(1.0)

if __name__ == '__main__':
 try:
  rospy.init_node('mailbot_1')
  result = example()
  rospy.spin()
 except rospy.ROSInterruptException:
  print "program interrupted before completion"
