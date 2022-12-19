import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='fov',
            default_value='60'
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'uol_turtlebot_simulator'), 'launch/turtlebot_world.launch.py')
            ),
            launch_arguments={
                'world_file': get_package_share_directory('uol_turtlebot_simulator') + '/worlds/maze3.world',
                'fov': launch.substitutions.LaunchConfiguration('fov')
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
