import rospy
from nav_msgs.msg import Odometry

class MySubscriber:

    def __init__(self):
        rospy.init_node('mySubscriber')
        self.pub = rospy.Subscriber('/odom', Odometry, self.callback)

    def callback(self, msg):
        print('I received this message: %s' % msg)


myP = MySubscriber()
rospy.spin()
