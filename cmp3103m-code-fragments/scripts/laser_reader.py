#!/usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan


class laser_reader:

    def __init__(self):
        self.laser_sub = rospy.Subscriber("/scan",
                                          LaserScan,
                                          self.callback)

    def callback(self, data):
        min_range = data.range_max
        print len(data.ranges)
        for v in data.ranges[300:340]:
            if v < min_range:
                min_range = v
        print min_range

ic = laser_reader()
rospy.init_node('laser_reader')
rospy.spin()
