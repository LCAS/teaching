#! /usr/bin/env python
import roslib; roslib.load_manifest('asp_session_files')
import rospy
from rosoclingo.msg import ROSoClingoOut, ROSoClingoIn

topic = rospy.Publisher('/ROSoClingo/in', ROSoClingoIn,latch=True)

#---------------------------------------------------------------------
# Services
#---------------------------------------------------------------------

#Black: 0
#Red:   1
#Green: 2
#Blue:  3
#Purple:4
#Yellow:5
#None: -1

location = {"turtlebot_1" : "5"}
qr_value = {"1" : "1", "2" : "-1", "3" : "2", "4" : "3", "5" : "-1",  "6" : "4", "7" : "3", "8" : "4", "9" : "6", "10" : "5", "11" : "6"}

#---------------------------------------------------------------------
# Services
#---------------------------------------------------------------------

def beep(robot):
	print "beep", robot, location[robot]
	return "success"

def move_robot(robot,waypoint):
	print "move", robot, waypoint
	location[robot] = waypoint
	return "success"

def get_qr(robot):
	value = qr_value[location[robot]]
	print "get_qr", robot, value
	return value

#---------------------------------------------------------------------
# Callback
#---------------------------------------------------------------------

def rosoclingoout_callback(message):
	if message.id == "turtlebot_1"	or message.id == "turtlebot_2":
#		if message.action[0:4] == "move":	value = move_robot(message.id,placemap[message.action[5:-1]])
		if message.action[0:4] == "move":	value = move_robot(message.id,message.action[5:-1])
		elif message.action[0:4] == "scan":	value = get_qr(message.id)
		elif message.action[0:4] == "beep":	value = beep(message.id)
		else:
			print "Invalid message:", message
			value = "failure"
		topic.publish(ROSoClingoIn(message.id,str(value)))
	else:
		print "Unknown robot:", message.id
	return None

#---------------------------------------------------------------------
# Main
#---------------------------------------------------------------------

if __name__ == '__main__':
        rospy.init_node('interface_winterschool')
	rospy.Subscriber("/ROSoClingo/out",ROSoClingoOut,rosoclingoout_callback)
	rospy.spin()
