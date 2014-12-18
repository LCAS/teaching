#! /usr/bin/env python

import roslib; roslib.load_manifest('rosoclingo_examples')
from rosoclingo.msg import *
import rospy
import actionlib

client = actionlib.ActionClient('ROSoClingo',ROSoClingoAction)

def send_goal(id):
 goal = ROSoClingoGoal(str(id))
 print "send " + str(id)
 return client.send_goal(goal)

def cancel_goal(goal):
 print "cancel " + str(goal)
 goal.cancel()

def example():
 handlers = []
 client.wait_for_server()
 handlers.append(send_goal(1))
 rospy.sleep(2.0)
 handlers.append(send_goal(2))
 rospy.sleep(2.0)
 handlers.append(send_goal(3))
 rospy.sleep(2.0)
 cancel_goal(handlers[2])
 rospy.sleep(2.0)
 cancel_goal(handlers[1])
 rospy.sleep(2.0)
 cancel_goal(handlers[0])

if __name__ == '__main__':
    try:
        rospy.init_node('minimal_1')
        result = example()
	rospy.spin()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
