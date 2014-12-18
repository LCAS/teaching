#! /usr/bin/env python

import roslib; roslib.load_manifest('asp_session_files')
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
 question = 'Please enter a request \'r [1-6]\' or cancel \'c ID\': '
 input = raw_input(question)
 while (input != ""):
  inlist = input.split(" ")
  if len(inlist) == 2 and inlist[0] == "c":
   handlers[int(inlist[1])].cancel()
   print "cancel " + inlist[1]
  elif len(inlist) == 2 and inlist[0] == "r":
   handlers.append(send_goal(inlist[1]))
   print "request ID: " + str((len(handlers)-1))
  else:
   print "input ignored"
  input = raw_input(question)

if __name__ == '__main__':
 try:
  rospy.init_node('winterschool_manual')
  example()
 except rospy.ROSInterruptException:
  print "program interrupted before completion"
