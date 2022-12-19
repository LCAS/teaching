import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='world_name',
            default_value=get_package_share_directory(
                'uol_turtlebot_simulator') + '/worlds/labC.world'
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'gazebo_ros'), 'launch/empty_world.launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'debug': 'false',
                'gui': 'true',
                'world_name': launch.substitutions.LaunchConfiguration('world_name')
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
