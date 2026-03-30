"""Classic launch file for spacenav_node."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='spacenav',
            executable='spacenav_node',
            name='spacenav_node',
            output='screen',
        ),
    ])
