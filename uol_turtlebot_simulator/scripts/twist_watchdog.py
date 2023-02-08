#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

class SendEmptyWatchdog(Node):
    def __init__(self, datatype, topic, timeout=0.5, send_value=None):
        super().__init__('TwistWatchDog')  

        self.timeout = timeout
        self.datatype = datatype

        if send_value is None:
            self.send_value = datatype()
        else:
            self.send_value = send_value

        self.create_subscription(datatype, topic, self.message_callback, 1)
        self.twist_publisher = self.create_publisher(datatype, topic, 1)
        self.timer = self.create_timer(self.timeout, self.timer_callback)

        self.timeout_triggered = False

    def message_callback(self, msg):
        self.get_logger().debug("SendEmptyWatchdog{}: Message Callback".format(self.datatype))
        if self.timeout_triggered:
            self.timeout_triggered = False
        else:
            if self.timer:
                self.timer.reset()
            else:
                self.timer = self.create_timer(self.timeout, self.timer_callback)

    def timer_callback(self):
        self.get_logger().warn("SendEmptyWatchdog{}: Timeout Triggered. Received no message for {} seconds".format(self.datatype, self.timeout))

        self.timeout_triggered = True
        self.twist_publisher.publish(self.send_value)
        self.timer.destroy()
        self.timer = None


def main(args=None):
    # always run "init()" first
    rclpy.init()

    # let's catch some exceptions should they happen
    try:
        twist_watchdog = SendEmptyWatchdog(Twist, "cmd_vel")

        # tell ROS to run this node until stopped (by [ctrl-c])
        rclpy.spin(twist_watchdog)

        # once stopped, tidy up
        node.destroy_node()
        rclpy.shutdown()

    except KeyboardInterrupt:
        print('Node interrupted')

    finally:
        # always print when the node has terminated
        print("Node terminated")


if __name__ == '__main__':
    main()