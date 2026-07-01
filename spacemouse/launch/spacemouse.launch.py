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
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    dashboard_port = LaunchConfiguration('dashboard_port')
    enable_pose = LaunchConfiguration('enable_pose')
    pose_frequency = LaunchConfiguration('pose_frequency')
    max_trans_speed = LaunchConfiguration('max_trans_speed')
    max_rot_speed = LaunchConfiguration('max_rot_speed')
    integration_frame = LaunchConfiguration('integration_frame')
    publish_curr_pose = LaunchConfiguration('publish_curr_pose')
    publish_delta_pose = LaunchConfiguration('publish_delta_pose')

    return LaunchDescription([
        DeclareLaunchArgument(
            'dashboard_port', default_value='',
            description='If set (e.g. 8080), also launch the web dashboard on '
                        'this HTTP port. Empty (default) = driver only.'),
        DeclareLaunchArgument(
            'enable_pose', default_value='true',
            description='Start the pose_node (curr_pose/delta_pose publisher).'),
        DeclareLaunchArgument(
            'pose_frequency', default_value='100.0',
            description='Pose publish rate in Hz.'),
        DeclareLaunchArgument(
            'max_trans_speed', default_value='0.1',
            description='Translation speed (m/s) at axis value 1.'),
        DeclareLaunchArgument(
            'max_rot_speed', default_value='1.0',
            description='Rotation speed (rad/s) at axis value 1.'),
        DeclareLaunchArgument(
            'integration_frame', default_value='world',
            description="Pose accumulation frame: 'body' or 'world'."),
        DeclareLaunchArgument(
            'publish_curr_pose', default_value='true',
            description='Publish spacemouse/curr_pose at launch '
                        '(also toggleable at runtime).'),
        DeclareLaunchArgument(
            'publish_delta_pose', default_value='true',
            description='Publish spacemouse/delta_pose at launch '
                        '(also toggleable at runtime).'),

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
                'num_buttons': 15,
            }],
        ),

        # pose integrator — core "pose output" function (dashboard-independent)
        Node(
            package='spacemouse',
            executable='pose_node',
            name='pose_node',
            output='screen',
            condition=IfCondition(enable_pose),
            parameters=[{
                'publish_frequency': ParameterValue(
                    pose_frequency, value_type=float),
                'max_trans_speed': ParameterValue(
                    max_trans_speed, value_type=float),
                'max_rot_speed': ParameterValue(
                    max_rot_speed, value_type=float),
                'integration_frame': ParameterValue(
                    integration_frame, value_type=str),
                'publish_curr_pose': ParameterValue(
                    publish_curr_pose, value_type=bool),
                'publish_delta_pose': ParameterValue(
                    publish_delta_pose, value_type=bool),
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
                'pose_node_name': 'pose_node',
            }],
        ),
    ])
