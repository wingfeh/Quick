from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mega3_control',
            executable='m3relay',
            name='m3relay_node',
            output='screen'
        ),
        Node(
            package='mega3_control',
            executable='m3sensor',
            name='m3sensor_node',
            output='screen'
        ),
        Node(
            package='mega3_control',
            executable='mega3_gui',
            name='mega3_gui_node',
            output='screen'
        ),
    ])
