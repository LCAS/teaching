import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class Chatter:

    def __init__(self):
        rospy.init_node('chatter')
        self.publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.laser_cb)

    def laser_cb(self, laser_msg):
        if laser_msg.ranges[320] < 1.0:
            t = Twist()
            t.angular.z = 1.0
            self.publisher.publish(t)
        else:
            t = Twist()
            t.linear.x = 1.0
            self.publisher.publish(t)

    def run(self):
        rospy.spin()

c = Chatter()
c.run()
