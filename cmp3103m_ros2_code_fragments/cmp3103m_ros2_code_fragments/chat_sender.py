#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class Chatter(Node):
    """ a simple "chatter" that publishes String messages on a topic.
    
        Once this is running, you can use `ros2 topic echo /msgs` to see the messages published on the topic.
    """

    def __init__(self):
        """ Initialise the Node. """
        # calling the constructor of the super class with the name of the node
        super().__init__('chatter')  

        # creating a ROS2 Publisher, for type "String" and topic name "/msgs"
        # the third argument is the length of the queue, i.e., only the last message is queued here
        self.publisher = self.create_publisher(String, '/msgs', 1)
        timer_period = 1  # seconds

        # Creating a timer that will trigger the "run_step" callback every second (event-driven programming)
        self.timer = self.create_timer(timer_period, self.run_step)

        # let's create a counter object in this Node, to show how to work with member variables in Python
        self.counter = 0

    def run_step(self):
        """ the main function that is run by a timer frequently """
        # First create a ROS2 String object. See `ros2 interface show std_msgs/msg/String` to see 
        # the interface / data type definition
        data_object = String()
        
        # put some data into the create object
        data_object.data = 'Hi! counter=%d' % self.counter

        # increase the counter by 1
        self.counter += 1

        print("I'm going to publish %s" % data_object)

        # now we are ready to publish the data
        self.publisher.publish(data_object)

def main(args=None):
    # always run "init()" first
    rclpy.init()

    # let's catch some exceptions should they happen
    try:
        # create the Chatter object
        node = Chatter()

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
