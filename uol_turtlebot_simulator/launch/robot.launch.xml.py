import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='x',
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='y',
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='z',
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='robot_name',
            default_value='turtlebot'
        ),
        launch.actions.DeclareLaunchArgument(
            name='move_base',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='gmapping',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='initial_pose_x',
            default_value=launch.substitutions.LaunchConfiguration('x')
        ),
        launch.actions.DeclareLaunchArgument(
            name='initial_pose_y',
            default_value=launch.substitutions.LaunchConfiguration('y')
        ),
        launch.actions.DeclareLaunchArgument(
            name='initial_pose_a',
            default_value='0.0'
        ),
        launch_ros.actions.Node(
            package='gazebo_ros',
            executable='spawn_model',
            name=launch.substitutions.LaunchConfiguration('robot_name')
        ),
        launch_ros.actions.Node(
            package='nodelet',
            executable='nodelet',
            name=launch.substitutions.LaunchConfiguration('robot_name')
        ),
        launch_ros.actions.Node(
            package='nodelet',
            executable='nodelet',
            name='cmd_vel_mux',
            parameters=[
                {
                    'yaml_cfg_file': get_package_share_directory('turtlebot_bringup') + '/param/mux.yaml'
                }
            ]
        ),
        launch_ros.actions.Node(
            package='nodelet',
            executable='nodelet',
            name='bumper2pointcloud',
            parameters=[
                {
                    'pointcloud_radius': '0.24'
                }
            ]
        ),
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                {
                    'publish_frequency': '30.0'
                },
                {
                    'tf_prefix': launch.substitutions.LaunchConfiguration('robot_name')
                }
            ]
        ),
        launch_ros.actions.Node(
            package='nodelet',
            executable='nodelet',
            name=launch.substitutions.LaunchConfiguration('robot_name')
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
                    'output_frame_id': launch.substitutions.LaunchConfiguration('robot_name')
                },
                {
                    'range_min': '0.45'
                }
            ]
        ),
        launch_ros.actions.Node(
            package='nodelet',
            executable='nodelet',
            name=launch.substitutions.LaunchConfiguration('robot_name'),
            parameters=[
                get_package_share_directory(
                    'turtlebot_bringup') + '/param/defaults/smoother.yaml'
            ]
        ),
        launch_ros.actions.Node(
            package='nodelet',
            executable='nodelet',
            name=launch.substitutions.LaunchConfiguration('robot_name'),
            parameters=[
                get_package_share_directory(
                    'turtlebot_bringup') + '/param/defaults/smoother.yaml'
            ]
        ),
        launch_ros.actions.Node(
            package='nodelet',
            executable='nodelet',
            name=launch.substitutions.LaunchConfiguration('robot_name')
        ),
        launch_ros.actions.Node(
            package='robot_pose_ekf',
            executable='robot_pose_ekf',
            name='robot_pose_ekf',
            parameters=[
                {
                    'freq': '30.0'
                },
                {
                    'sensor_timeout': '1.0'
                },
                {
                    'publish_tf': 'true'
                },
                {
                    'odom_used': 'true'
                },
                {
                    'imu_used': 'true'
                },
                {
                    'vo_used': 'false'
                },
                {
                    'output_frame': 'odom'
                }
            ]
        ),
        launch_ros.actions.Node(
            package='move_base',
            executable='move_base',
            name='move_base',
            output='screen',
            parameters=[
                get_package_share_directory(
                    'uol_turtlebot_simulator') + '/param/local_costmap_params.yaml',
                get_package_share_directory(
                    'uol_turtlebot_simulator') + '/param/global_costmap_params.yaml',
                get_package_share_directory(
                    'uol_turtlebot_simulator') + '/param/base_local_planner_params.yaml',
                get_package_share_directory(
                    'uol_turtlebot_simulator') + '/param/move_base_params.yaml'
            ]
        ),
        launch_ros.actions.Node(
            package='gmapping',
            executable='slam_gmapping',
            name='gmapping',
            parameters=[
                {
                    'base_frame': launch.substitutions.LaunchConfiguration('robot_name')
                },
                {
                    'map_frame': '/map'
                },
                {
                    'odom_frame': launch.substitutions.LaunchConfiguration('robot_name')
                },
                {
                    'minimumScore': '50'
                },
                {
                    'particles': '50'
                },
                {
                    'linearUpdate': '1'
                },
                {
                    'angularUpdate': '0.5'
                }
            ]
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'uol_turtlebot_simulator'), 'launch/amcl/amcl.launch.xml.py')
            ),
            launch_arguments={
                'initial_pose_x': launch.substitutions.LaunchConfiguration('initial_pose_x'),
                'initial_pose_y': launch.substitutions.LaunchConfiguration('initial_pose_y'),
                'initial_pose_a': launch.substitutions.LaunchConfiguration('initial_pose_a'),
                'robot_name': launch.substitutions.LaunchConfiguration('robot_name')
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
