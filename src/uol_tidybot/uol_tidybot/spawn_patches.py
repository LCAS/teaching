#!/usr/bin/env python3

"""
Node to spawn patches at FIXED specific locations (≥2m apart) based on world_complexity.
- simple: 2 red patches at predefined positions
- medium: 1 red + 1 blue patch at predefined positions  
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
import time
import numpy as np

class RedPatchSpawner(Node):
    def __init__(self):
        super().__init__("red_patch_spawner")
        self.declare_parameter("arena_center_x", 0.0)
        self.declare_parameter("arena_center_y", 0.0)
        self.declare_parameter("patch_size_x", 0.8)
        self.declare_parameter("patch_size_y", 0.8)
        self.declare_parameter("patch_height", 0.00001)
        self.declare_parameter("patch_mass", 10000)
        self.declare_parameter("service_name", "/spawn_entity")
        self.declare_parameter("task_complexity", "low_complexity")

        # Get parameters
        self.center_x = self.get_parameter("arena_center_x").value
        self.center_y = self.get_parameter("arena_center_y").value
        self.size_x = self.get_parameter("patch_size_x").value
        self.size_y = self.get_parameter("patch_size_y").value
        self.height = self.get_parameter("patch_height").value
        self.mass = self.get_parameter("patch_mass").value
        self.srv_name = self.get_parameter("service_name").value
        self.task_complexity = self.get_parameter("task_complexity").value.lower()

        # Unique session ID
        self.session_id = str(int(time.time() * 1000))[-6:]

        # FIXED POSITIONS (all ≥2m apart, relative to arena center)
        # simple: uses first 2 positions
        # medium: uses first 2 positions (same reliable spots)
        self.fixed_positions = [
            (-0.7, 0.55),   # Position 0: NW quadrant (dist to 1: ~3.0m)
            (0.7, -0.5),   # Position 1: SE quadrant (dist to 0: ~3.0m)  
        ]

        # Configure patches
        if self.task_complexity == "low_complexity":
            self.patches = [("red", 1), ("blue", 0)]
        elif self.task_complexity == "medium_complexity":
            self.patches = [("red", 1), ("blue", 1)]
        else:
            self.get_logger().warn(f"Unknown task_complexity '{self.task_complexity}', defaulting to simple")
            self.patches = [("red", 2)]

        self.get_logger().info(f"Session {self.session_id}: Spawning {self.patches} for '{self.task_complexity}'")
        self.get_logger().info(f"Fixed positions relative to center ({self.center_x}, {self.center_y}): {self.fixed_positions}")

        # Service setup
        self.srv = self.create_client(SpawnEntity, self.srv_name)
        while not self.srv.wait_for_service(1.0):
            self.get_logger().warn(f"Waiting for {self.srv_name} service...")
        self.get_logger().info(f"Connected to {self.srv_name}")

        self.spawn_patches()

    def get_material_xml(self, color):
        if color == "red":
            return """
            <material>
              <ambient>1.0 0.0 0.0 1.0</ambient>
              <diffuse>1.0 0.0 0.0 1.0</diffuse>
              <specular>0.1 0.1 0.1 1.0</specular>
            </material>"""
        elif color == "blue":
            return """
            <material>
              <ambient>0.0 0.0 1.0 1.0</ambient>
              <diffuse>0.0 0.0 1.0 1.0</diffuse>
              <specular>0.1 0.1 0.1 1.0</specular>
            </material>"""
        else:
            self.get_logger().error(f"Unknown color '{color}', using red")
            return self.get_material_xml("red")

    def spawn_patches(self):
        patch_id = 0
        pos_idx = 0
        
        for color, count in self.patches:
            for _ in range(count):
                # Use fixed positions (offset from arena center)
                dx, dy = self.fixed_positions[pos_idx]
                
                x = self.center_x + dx
                y = self.center_y + dy

                if self.task_complexity == "low_complexity":
                    x = x+np.random.uniform(-0.4, 0.1)
                    y = y+np.random.uniform(0.4, 0.1)

                unique_name = f"{color}_patch_{self.session_id}_{patch_id}"
                material_xml = self.get_material_xml(color)

                xml = f"""
                <?xml version="1.0" ?>
                <sdf version="1.6">
                  <model name="{unique_name}">
                    <static>true</static>
                    <pose>{x} {y} {self.height/2.0} 0 0 0</pose>
                    <link name="link">
                      <collision name="collision">
                        <geometry>
                          <box>
                            <size>{self.size_x} {self.size_y} {self.height}</size>
                          </box>
                        </geometry>
                      </collision>
                      <visual name="visual">
                        <geometry>
                          <box>
                            <size>{self.size_x} {self.size_y} {self.height}</size>
                          </box>
                        </geometry>
                        {material_xml}
                      </visual>
                    </link>
                  </model>
                </sdf>
                """

                req = SpawnEntity.Request()
                req.name = unique_name
                req.xml = xml
                req.initial_pose = Pose()
                req.initial_pose.position.z = self.height / 2.0
                req.reference_frame = "world"

                future = self.srv.call_async(req)
                rclpy.spin_until_future_complete(self, future)
                if future.result() is not None:
                    self.get_logger().info(f"Spawned {color} patch '{unique_name}' at ({x:.2f}, {y:.2f})")
                else:
                    self.get_logger().error(f"Failed to spawn {color} patch '{unique_name}'")

                patch_id += 1
                pos_idx += 1

        self.get_logger().info("All patches spawned at fixed positions. Shutting down.")
        self.destroy_node()

def main():
    rclpy.init()
    spawner = RedPatchSpawner()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
