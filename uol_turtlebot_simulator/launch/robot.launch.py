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
                'uol_turtlebot_simulator') + '/maps/object-search-arena.yaml'
        ),
        launch.actions.DeclareLaunchArgument(
            name='move_base',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='gmapping',
            default_value='false'
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
                    'uol_turtlebot_simulator'), 'launch/single-robot.launch.xml.py')
            ),
            launch_arguments={
                'move_base': launch.substitutions.LaunchConfiguration('move_base'),
                'gmapping': launch.substitutions.LaunchConfiguration('gmapping')
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
