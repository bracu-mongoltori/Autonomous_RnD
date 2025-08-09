#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_control',
            executable='drone_controller',
            name='drone_controller',
            output='screen',
            parameters=[
                {'connection_string': '/dev/ttyACM0'},  # Change this to your connection
                {'baud_rate': 115200}
            ]
        ),
        Node(
            package='drone_control',
            executable='joystick_interface',
            name='joystick_interface',
            output='screen'
        ),
        Node(
            package='drone_control',
            executable='drone_test_client',
            name='drone_test_client',
            output='screen'
        )
    ])
