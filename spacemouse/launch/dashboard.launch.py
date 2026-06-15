"""Launch the web dashboard node."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('http_port', default_value='8080',
                              description='HTTP server port for the web UI'),
        Node(
            package='spacemouse',
            executable='dashboard_node',
            name='dashboard_node',
            output='screen',
            parameters=[{
                'http_port': LaunchConfiguration('http_port'),
            }],
        ),
    ])
