#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Launch file for Mega2 control system"""
    
    return LaunchDescription([
        # M2 Relay Node
        Node(
            package='mega2_control',
            executable='m2relay',
            name='m2relay',
            output='screen',
            parameters=[],
            remappings=[]
        ),
        
        # M2 Sensor Node
        Node(
            package='mega2_control',
            executable='m2sensor',
            name='m2sensor',
            output='screen',
            parameters=[],
            remappings=[]
        ),
        
        # M2 GUI Node
        Node(
            package='mega2_control',
            executable='mega2_gui',
            name='mega2_gui',
            output='screen',
            parameters=[],
            remappings=[]
        ),
    ])
