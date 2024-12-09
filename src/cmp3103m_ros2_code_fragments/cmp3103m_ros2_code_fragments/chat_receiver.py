#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class ChatReceiver(Node):
    """ a simple "ChatReceiver" that publishes String messages on a topic.


    """

    def __init__(self):
        """ Initialise the Node. """
        # calling the constructor of the super class with the name of the node
        super().__init__('ChatReceiver')  

        # creating a ROS2 Subscriber, for type "String" and topic name "/msgs"
        # the forth argument is the length of the queue, i.e., only the last message is queued here
        self.create_subscription(String, '/msgs', self.callback, 1)

    def callback(self, msg):
        """ the main callback, triggered when a message is received.

            The `msg` field contains the actual ROS2 message object received.
        """
        
        # simply print the received message on the screen:
        print("I received this message: %s" % msg)

def main(args=None):
    # always run "init()" first
    rclpy.init()

    # let's catch some exceptions should they happen
    try:
        # create the ChatReceiver object
        node = ChatReceiver()

        # tell ROS to run this node until stopped (by [ctrl-c])
        rclpy.spin(node)

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
