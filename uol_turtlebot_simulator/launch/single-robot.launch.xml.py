import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='base',
            default_value=os.environ.get('TURTLEBOT_BASE', 'kobuki')
        ),
        launch.actions.DeclareLaunchArgument(
            name='battery',
            default_value='$(optenv TURTLEBOT_BATTERY /proc/acpi/battery/BAT0)'
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
            name='model',
            default_value=get_package_share_directory(
                'uol_turtlebot_simulator') + '/urdf/kobuki_hexagons_kinect.urdf.xacro'
        ),
        launch.actions.DeclareLaunchArgument(
            name='move_base',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='gmapping',
            default_value='false'
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
