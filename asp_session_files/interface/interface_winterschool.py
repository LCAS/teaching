#! /usr/bin/env python
import roslib; roslib.load_manifest('asp_session_files')
import rospy
from rosoclingo.msg import ROSoClingoOut, ROSoClingoIn
from services.srv import *
from std_msgs.msg import String
import actionlib
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction, MoveBaseActionFeedback
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from kobuki_msgs.msg import Sound
import math

topic = rospy.Publisher('/ROSoClingo/in', ROSoClingoIn,latch=True)

#---------------------------------------------------------------------
# A map from names to (x,y,z,t) waypoints
#---------------------------------------------------------------------

placemap = {
"1" : (-3.00, 1.00),
"2" : (-1.25,-0.50),
"3" : (-2.25,-2.50),
"4" : (-1.00, 1.25),
"5" : ( 1.00,-0.50),
"6" : (-0.50,-1.75),
"7" : ( 0.50, 1.00),
"8" : ( 0.50,-2.50),
"9" : ( 2.50,-1.50),
"10" : ( 3.00, 1.00),
"11" : ( 2.50,-2.75),
"12" : (-2.60,-1.25),
"13" : ( 3.00,-0.50)
}

rotationmap = {
"1" : 0.636244416237,
"3" : -2.77760386467,
"4" : 0.0,
"6" : 0.0,
"7" : 2.72584557533,
"8" : 2.80970883369,
"9" : -1.65704452991,
"10" : 0.0,
"11" : 1.80403387547,
}

#---------------------------------------------------------------------
# Services
#---------------------------------------------------------------------

def get_location(robot):
	service = robot + "/getLocation"
	rospy.wait_for_service(service)
	try:
		my_get_location = rospy.ServiceProxy(service,getLocation)
		resp = my_get_location(0)
		return resp
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def get_status(robot):
	service = robot + "/getStatus"
	rospy.wait_for_service(service)
	try:
		my_get_status = rospy.ServiceProxy(service,getStatus)
		resp = my_get_status(0)
		return resp
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def doRotate(robot):
	service = robot + "/rotate"
	rospy.wait_for_service(service)
	try:
		my_rotate = rospy.ServiceProxy(service,rotate)
		resp = my_rotate(1)
		return resp
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def beep(robot):
	publisher = robot + "/mobile_base/commands/sound"
	soundtopic = rospy.Publisher(publisher,Sound,latch=True)
	soundtopic.publish(Sound(6))
	return "success"

def get_qr(robot):
	service = robot + "/getQR"
	rospy.wait_for_service(service)
	try:
		my_get_qr = rospy.ServiceProxy(service,getQR)
		resp = doRotate(robot)
		status = get_status(robot).status
                while(status == 1): 
			rospy.sleep(0.5)
			status = get_status(robot).status
		resp = my_get_qr(0)
		qrcode = resp.qrcode 
		if qrcode == 0: qrcode = 4
		return qrcode
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def get_quaternion_from_position(a,b,x,y):
	if x-a >= 0: sign = 1 
	else: sign = -1
	v = y-b
	r = math.sqrt((x-a)**2 + (y-b)**2)
	deg = math.degrees(math.asin(v / r))
	z = deg / 180 * 3.14 * sign
	if x-a >= 0: z = z
	else: z = 3.14 + z
	return quaternion_from_euler(0.0,0.0,z)

def move_robot(robot,target):
	target = placemap[target]
	current = get_location(robot)
	quanternion = get_quaternion_from_position(current.x,current.y,target[0],target[1])
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = target[0]
	goal.target_pose.pose.position.y = target[1]
	goal.target_pose.pose.orientation.z = quanternion[2]
	goal.target_pose.pose.orientation.w = quanternion[3]
	actionlibName = robot + '/move_base'
	client = actionlib.SimpleActionClient(actionlibName, MoveBaseAction)
	client.wait_for_server()
	client.send_goal(goal)
	client.wait_for_result(rospy.Duration.from_sec(45.0))
	current = get_location(robot)
	if abs(current.x - target[0]) < 0.5 and abs(current.y - target[1]) < 0.5: return "successful"
	else: return "failure"

#---------------------------------------------------------------------
# Callback
#---------------------------------------------------------------------

def rosoclingoout_callback(message):
	if message.id == "turtlebot_1"	or message.id == "turtlebot_2":
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
