import rospy
from geometry_msgs.msg import Twist

class MyPublisher:

    def __init__(self):
        rospy.init_node('mypublisher')
        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)

    def run(self):
        while not rospy.is_shutdown():
            s = Twist()
            s.angular.z = 0.5
            print('debug: %s' % s)
            self.pub.publish(s)
            rospy.sleep(.3)


myP = MyPublisher()
myP.run()
#rospy.spin()
