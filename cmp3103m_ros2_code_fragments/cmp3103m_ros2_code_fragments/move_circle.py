#!/usr/bin/env python

# An example of TurtleBot 3 drawing a 1 meter square.
# Written for humble

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from math import radians

class MoveInACircle(Node):
    def __init__(self):
        super().__init__('move_circle')        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.move_cmd = Twist()
        self.move_cmd.linear.x = 0.05   # m/s
        self.move_cmd.angular.z = radians(10); #10 deg/s in radians/s

    def timer_callback(self):
        self.cmd_vel_pub.publish(self.move_cmd)
 

def main(args=None):
    rclpy.init()
    try:
        node = MoveInACircle()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

    except Exception as e:
        print("node terminated. %s" , e)


if __name__ == '__main__':
    main()
