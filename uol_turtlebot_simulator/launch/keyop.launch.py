import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='turtlebot_teleop',
            executable='turtlebot_teleop_key',
            name='turtlebot_teleop_keyboard',
            output='screen',
            parameters=[
                {
                    'scale_linear': '0.5'
                },
                {
                    'scale_angular': '1.5'
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
