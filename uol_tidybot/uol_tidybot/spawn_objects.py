# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from random import normalvariate, uniform
from uuid import uuid4

import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node


class ObjectSpawner(Node):
    """
    A class for spawning objects in a ROS2 environment.
       
    """

    def __init__(self):
        super().__init__('ObjectSpawner')
        # declare a new ROS2 boolean parameter named "red"
        self.declare_parameter('red', False, descriptor=ParameterDescriptor(description='Use red dice model instead of the default white dice model. Default is False.'))
        self.declare_parameter('n_objects', 10, descriptor=ParameterDescriptor(description='Number of objects to spawn. Default is 10.'))
        self.declare_parameter('cx', 0.0, descriptor=ParameterDescriptor(description='Center of the object distribution (x coordinate in m). Default is 0.0.'))
        self.declare_parameter('cy', 0.0, descriptor=ParameterDescriptor(description='Center of the object distribution (y coordinate in m). Default is 0.0.'))
        self.declare_parameter('spread', 1.0, descriptor=ParameterDescriptor(description='Spread of the object distribution (in m). Default is 1.0.'))
        self.declare_parameter('normal_distribution', False, descriptor=ParameterDescriptor(description='Use normal distribution instead of uniform distribution. Default is False.'))

        # Create a client for the 'spawn_entity' service
        self.cli = self.create_client(SpawnEntity, 'spawn_entity')

        # Wait for the service to become available
        while not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('service not available, waiting again...')

        # Create a request object for the 'spawn_entity' service
        self.req = SpawnEntity.Request()

        # Determine the path to the SDF file based on the value of the 'red' parameter
        if self.get_parameter('red').value:
            self.sdf_file = os.path.join(
                get_package_share_directory('uol_tidybot'), 
                'models','dice_simple_red','model.sdf')
        else:
            self.sdf_file = os.path.join(
                get_package_share_directory('uol_tidybot'), 
                'models','dice_simple','model.sdf')

        # Read the contents of the SDF file
        with open(self.sdf_file, mode='r') as file:
            self.sdf = file.read()

        self.get_logger().info('loaded')

    def place_object(self, x, y, name=None):
            """
            Places an object at the specified coordinates.

            Args:
                x (float): X coordinate of the object.
                y (float): Y coordinate of the object.
                name (str, optional): Name of the object. If not provided, a random UUID will be used as the name.

            """
            # Set the XML description of the object from the loaded SDF file
            self.req.xml = self.sdf

            # Set the name of the object. Generate a random UUID if no name is provided 
            self.req.name = str(uuid4()) if name is None else name

            # Set the initial position of the object
            self.req.initial_pose.position.x = x
            self.req.initial_pose.position.y = y
            self.req.initial_pose.position.z = 1.0

            self.get_logger().debug('req: %s' % self.req)

            # Call the service asynchronously (non-blocking)
            future = self.cli.call_async(self.req)

            # Log that the call has been made and wait for the result
            self.get_logger().debug('called, waiting')
            # spin_until_future_complete() will block until the future has a result, i.e., until the object has been spawned
            rclpy.spin_until_future_complete(self, future)

            self.get_logger().debug(future.result())
            self.get_logger().debug('leave')

    def place_objects(self):
            """
            Places multiple objects based on the specified ROS parameters.
            """
            # Get the number of objects to be placed
            N = self.get_parameter('n_objects').value
            # Get the center x-coordinate
            cx = self.get_parameter('cx').value
            # Get the center y-coordinate
            cy = self.get_parameter('cy').value
            # Get the spread value
            spread = self.get_parameter('spread').value
            # Check if normal distribution should be used
            use_normal_distribution = self.get_parameter('normal_distribution').value

            # log all the parameters to the info log level
            self.get_logger().info(' -> n_objects: %d' % N)
            self.get_logger().info(' -> cx: %f' % cx)
            self.get_logger().info(' -> cy: %f' % cy)
            self.get_logger().info(' -> spread: %f' % spread)
            self.get_logger().info(' -> normal_distribution? %s' % use_normal_distribution)

            # Iterate over the number of objects
            for i in range(0,N):
                # Check if normal distribution should be used
                if use_normal_distribution:
                    # Generate x and y coordinates using normal distribution
                    x = normalvariate(cx, spread)
                    y = normalvariate(cy, spread)
                else:
                    # Generate x and y coordinates using uniform distribution
                    x = uniform(cx-spread, cx+spread)
                    y = uniform(cy-spread, cy+spread)
                # Place the object at the generated coordinates
                self.get_logger().info('place object at (%f, %f)' % (x, y))
                self.place_object(x, y)



def main(args=None):
    """
    Main function to spawn objects in the environment. Here a typical way to run this:

    `ros2 run  uol_tidybot generate_objects --ros-args -p red:=true -p n_objects:=10`

    Args:
        args (list): List of command-line arguments (default: None)
    """
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create an instance of the ObjectSpawner node
    spawner = ObjectSpawner()

    # Place objects in the environment
    spawner.place_objects()

    # Uncomment the following line to start spinning the node
    # rclpy.spin(spawner)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    spawner.destroy_node()

    # Shutdown the ROS 2 Python client library
    rclpy.shutdown()


if __name__ == '__main__':
    main()