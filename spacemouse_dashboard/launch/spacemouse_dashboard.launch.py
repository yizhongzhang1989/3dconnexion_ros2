"""Launch both spacenav driver and web dashboard."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('spacemouse_dashboard')

    return LaunchDescription([
        DeclareLaunchArgument('http_port', default_value='8080'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'spacemouse.launch.py')
            ),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'dashboard.launch.py')
            ),
            launch_arguments={
                'http_port': LaunchConfiguration('http_port'),
            }.items(),
        ),
    ])
