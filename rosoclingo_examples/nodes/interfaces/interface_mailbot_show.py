#! /usr/bin/env python

import rospy
from rosoclingo.msg import ROSoClingoOut, ROSoClingoIn
import sys

topic = rospy.Publisher('/ROSoClingo/in', ROSoClingoIn,latch=True)

def rosoclingoout_callback(message):
 global size, position
 out_message = ROSoClingoIn()
 out_message.id = message.id
 out_message.value = "success"
 if message.action[0:6] == "pickup":
  show_environment(size,position,"^","pickup package for: " + message.action[7:-1].split(",")[1])
 elif message.action[0:7] == "deliver":
  show_environment(size,position,"^","deliver package from: " + message.action[8:-1].split(",")[0])
 elif message.action[0:2] == "go":
  if message.action[3:-1] == "1":
   position = position + 1
   show_environment(size,position,">","go up")
  elif message.action[3:-1] == "-":
   position = position - 1
   show_environment(size,position,"<","go down")
  else:
   print "There is an error: unknown parameter for action go: " + message.action_args[0]
 else:
  print "There is an error: unknown action: " + message.action_name
 rospy.sleep(2.5)
 topic.publish(out_message)

def show_environment(size,position,symbol,message):
 for l in xrange(1,size+1):
  if l == position: print symbol,
  else: print l,
 print message

size = int(sys.argv[1])
position = int(sys.argv[2])
rospy.init_node('interface_mailbot_show')
show_environment(size,position,"^","")
rospy.Subscriber("/ROSoClingo/out",ROSoClingoOut,rosoclingoout_callback)
rospy.spin()
