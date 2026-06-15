"""Launch the SpaceMouse (spacenav driver), optionally with the web dashboard.

Always starts the ``spacenav`` driver node. If the optional ``dashboard_port``
argument is given (non-empty), the web dashboard node is also started on that
port::

    # driver only
    ros2 launch spacemouse spacemouse.launch.py

    # driver + dashboard on http://localhost:8080
    ros2 launch spacemouse spacemouse.launch.py dashboard_port:=8080
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    dashboard_port = LaunchConfiguration('dashboard_port')

    return LaunchDescription([
        DeclareLaunchArgument(
            'dashboard_port', default_value='',
            description='If set (e.g. 8080), also launch the web dashboard on '
                        'this HTTP port. Empty (default) = driver only.'),

        # spacenav driver — always launched
        Node(
            package='spacenav',
            executable='spacenav_node',
            name='spacenav_node',
            output='screen',
            parameters=[{
                'full_scale': 350.0,
                'zero_when_static': True,
                'static_count_threshold': 30,
                'static_trans_deadband': 0.1,
                'static_rot_deadband': 0.1,
            }],
        ),

        # web dashboard — only when dashboard_port is provided
        Node(
            package='spacemouse',
            executable='dashboard_node',
            name='dashboard_node',
            output='screen',
            condition=IfCondition(
                PythonExpression(["'", dashboard_port, "' != ''"])),
            parameters=[{
                'http_port': dashboard_port,
            }],
        ),
    ])
