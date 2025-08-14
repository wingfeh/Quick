#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Arduino relay controller node
        Node(
            package='mega5_control',
            executable='m5relay',
            name='m5relay',
            output='screen',
            emulate_tty=True,
        ),
        
        # GUI node
        Node(
            package='mega5_control',
            executable='mega5_gui',
            name='mega5_gui',
            output='screen',
            emulate_tty=True,
        ),
        
        # Sensor monitoring node
        Node(
            package='mega5_control',
            executable='m5sensor',
            name='m5sensor',
            output='screen',
            emulate_tty=True,
        ),
    ])
