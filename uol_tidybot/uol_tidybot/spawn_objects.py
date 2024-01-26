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

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from ament_index_python.packages import get_package_share_directory
import os
from uuid import uuid4
from random import normalvariate, uniform

class ObjectSpawner(Node):

    def __init__(self, sdf_file=None):
        super().__init__('ObjectSpawner')
        self.cli = self.create_client(SpawnEntity, 'spawn_entity')
        while not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SpawnEntity.Request()
        if sdf_file is None:
            self.sdf_file = os.path.join(
                get_package_share_directory('uol_tidybot'), 
                'models','dice_simple','model.sdf')
        else:
            self.sdf_file = sdf_file
        
        with open(self.sdf_file, mode='r') as file:
            self.sdf = file.read()
        self.get_logger().info('loaded')

    def place_object(self, x, y, name=None):
        self.req.xml = self.sdf
        self.req.name = str(uuid4()) if name is None else name
        #self.req.reference_frame = 'odom'
        self.req.initial_pose.position.x = x
        self.req.initial_pose.position.y = y
        self.req.initial_pose.position.z = 1.0
        self.get_logger().debug('req: %s' % self.req)
        future = self.cli.call_async(self.req)
        self.get_logger().debug('called, waiting')
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().debug(future.result())
        self.get_logger().debug('leave')

    def place_objects(self, N=20, cx=0.0, cy=0.0, spread=1.0):
        for i in range(0,N):
            x = uniform(cx-spread, cx+spread)
            y = uniform(cy-spread, cy+spread)
            #x = normalvariate(cx, spread)
            #y = normalvariate(cy, spread)
            self.get_logger().info('place object at (%f, %f)' % (x, y))
            self.place_object(x, y)




def main(args=None):
    rclpy.init(args=args)

    spawner = ObjectSpawner()
    spawner.place_objects()

    #rclpy.spin(spawner)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    spawner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()