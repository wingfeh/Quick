#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Arduino relay controller node
        Node(
            package='mega1_control',
            executable='m1relay',
            name='m1relay',
            output='screen',
            emulate_tty=True,
        ),
        
        # GUI node
        Node(
            package='mega1_control',
            executable='mega1_gui',
            name='mega1_gui',
            output='screen',
            emulate_tty=True,
        ),
        
        # Sensor monitoring node
        Node(
            package='mega1_control',
            executable='m1sensor',
            name='m1sensor',
            output='screen',
            emulate_tty=True,
        ),
    ])
