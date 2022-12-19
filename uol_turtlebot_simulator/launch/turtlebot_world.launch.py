import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='world_file',
            default_value=os.environ.get('TURTLEBOT_GAZEBO_WORLD_FILE')
        ),
        launch.actions.DeclareLaunchArgument(
            name='base',
            default_value=os.environ.get('TURTLEBOT_BASE', 'kobuki')
        ),
        launch.actions.DeclareLaunchArgument(
            name='battery',
            default_value='$(optenv TURTLEBOT_BATTERY /proc/acpi/battery/BAT0)'
        ),
        launch.actions.DeclareLaunchArgument(
            name='gui',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='stacks',
            default_value=os.environ.get('TURTLEBOT_STACKS', 'hexagons')
        ),
        launch.actions.DeclareLaunchArgument(
            name='3d_sensor',
            default_value=os.environ.get('TURTLEBOT_3D_SENSOR', 'kinect')
        ),
        launch.actions.DeclareLaunchArgument(
            name='fov',
            default_value='60'
        ),
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                {
                    'publish_frequency': '30.0'
                }
            ]
        ),
        launch_ros.actions.Node(
            package='nodelet',
            executable='nodelet',
            name='laserscan_nodelet_manager'
        ),
        launch_ros.actions.Node(
            package='nodelet',
            executable='nodelet',
            name='depthimage_to_laserscan',
            parameters=[
                {
                    'scan_height': '10'
                },
                {
                    'output_frame_id': '/camera_depth_frame'
                },
                {
                    'range_min': '0.001'
                },
                {
                    'range_max': '100'
                }
            ]
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'gazebo_ros'), 'launch/empty_world.launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'debug': 'false',
                'gui': launch.substitutions.LaunchConfiguration('gui'),
                'world_name': launch.substitutions.LaunchConfiguration('world_file')
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'uol_turtlebot_simulator'), 'launch/includes/kobuki.launch.xml.py')
            ),
            launch_arguments={
                'base': 'kobuki',
                'stacks': launch.substitutions.LaunchConfiguration('stacks'),
                '3d_sensor': 'kinect',
                'fov': launch.substitutions.LaunchConfiguration('fov')
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
