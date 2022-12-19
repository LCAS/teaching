import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='move_base',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='gmapping',
            default_value='false'
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'turtlebot_gazebo'), 'launch/turtlebot_world.launch.py')
            ),
            launch_arguments={
                'world_file': get_package_share_directory('uol_turtlebot_simulator') + '/worlds/object-search-training.world'
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'turtlebot_gazebo'), 'launch/amcl_demo.launch.py')
            ),
            launch_arguments={
                'map_file': get_package_share_directory('uol_turtlebot_simulator') + '/maps/object-search-arena.yaml'
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
