"""Launch the spacenav driver node."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
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
    ])
