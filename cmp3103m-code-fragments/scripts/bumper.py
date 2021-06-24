import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent


class Chatter:

    def __init__(self):
        rospy.init_node('chatter')
        self.publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.bumper_cb)
        self.obstacles = False

    def bumper_cb(self, laser_msg):
        if laser_msg.bumper != 1:
            return
        if laser_msg.state == 1:
            self.obstacles = True
        else: 
            self.obstacles = False

    def run(self):
        while not rospy.is_shutdown():
            t = Twist()
            if self.obstacles:
                t.angular.z = 1.0
            else:
                t.linear.x = 0.4
            self.publisher.publish(t)

c = Chatter()
c.run()
