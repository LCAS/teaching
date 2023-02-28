import rclpy

from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class FirstSub(Node):
    def __init__(self):
        super().__init__('firstsub')

        self.sub = self.create_subscription(
            LaserScan,"/scan",
            self.callback, 1)
            
        self.pub = self.create_publisher(String, '/warning', 1)

    def callback(self, data):
        for range in data.ranges:
            if range < 1.0:
                print("ALERT")
                str = String()
                str.data = "ALERT"
                self.pub.publish(str)

def main(args=None):
    rclpy.init()
    node = FirstSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()