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
 question = 'Please enter a request \'FROM TO\' or cancel \'ID\': '
 input = raw_input(question)
 while (input != ""):
  inlist = input.split(" ")
  if len(inlist) == 1:
   handlers[int(inlist[0])].cancel()
   print "cancel " + inlist[0]
  elif len(inlist) == 2:
   handlers.append(send_goal(inlist[0],inlist[1]))
   print "request ID: " + str((len(handlers)-1))
  else:
   print "input ignored"
  input = raw_input(question)

if __name__ == '__main__':
 try:
  rospy.init_node('mailbot_manual')
  result = example()
 except rospy.ROSInterruptException:
  print "program interrupted before completion"
