#!/usr/bin/env python

# An example of TurtleBot 3 drawing a 1 meter square.
# Written for humble

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from math import radians

class DrawASquare(Node):
    def __init__(self):
        super().__init__('drawasquare')        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.count = 0
        self.loops = 30 # 30 loops at 10 Hz, takes around 3s

    	# create two different Twist() variables.  One for moving forward.  One for turning 45 degrees.

        # let's go forward at 0.2 m/s
        self.move_cmd = Twist()
        self.move_cmd.linear.x = 1.0/3.0
	    # by default angular.z is 0 so setting this isn't required

        #let's turn at 45 deg/s
        self.turn_cmd = Twist()
        self.turn_cmd.linear.x = 0.0
        self.turn_cmd.angular.z = radians(90/3); #30 deg/s in radians/s

    def timer_callback(self):
        #two keep drawing squares.  Go forward for 2 seconds (10 x 5 HZ) then turn for 2 second
	    # go forward 1 m (5 seconds * 0.2 m / seconds)
        if (self.count // self.loops) % 2 == 0: # Even number
            self.get_logger().info("Going Straight")
            self.cmd_vel_pub.publish(self.move_cmd)
        else:
            self.get_logger().info("Turning")
            self.cmd_vel_pub.publish(self.turn_cmd)
        self.count += 1
        self.get_logger().info('count: %d' % self.count)
 

def main(args=None):
    rclpy.init()
    try:
        node = DrawASquare()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

    except Exception as e:
        print("node terminated. %s" , e)


if __name__ == '__main__':
    main()
