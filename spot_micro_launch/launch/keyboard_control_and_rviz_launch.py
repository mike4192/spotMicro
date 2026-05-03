import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    rviz_slam = LaunchConfiguration('rviz_slam')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_slam',
            default_value='false',
            description='Run RViz with SLAM visualization'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('spot_micro_keyboard_command'),
                'launch',
                'keyboard_command_launch.py'))),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('spot_micro_rviz'),
                'launch',
                'slam_launch.py')),
            condition=IfCondition(rviz_slam)),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('spot_micro_rviz'),
                'launch',
                'show_model_launch.py')),
            condition=UnlessCondition(rviz_slam)),
    ])
