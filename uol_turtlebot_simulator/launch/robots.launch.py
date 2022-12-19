import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='map_file',
            default_value=get_package_share_directory(
                'uol_turtlebot_simulator') + '/maps/labC-cropped.yaml'
        ),
        launch_ros.actions.Node(
            package='map_server',
            executable='map_server',
            name='map_server',
            parameters=[
                {
                    'frame_id': '/map'
                }
            ]
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'uol_turtlebot_simulator'), 'launch/multi-robots.launch.xml.py')
            )
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
