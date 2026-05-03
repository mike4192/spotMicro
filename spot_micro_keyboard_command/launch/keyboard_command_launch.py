import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    run_plot = LaunchConfiguration('run_plot')
    run_rviz = LaunchConfiguration('run_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'run_plot',
            default_value='false',
            description='Run python plotting node'),
        DeclareLaunchArgument(
            'run_rviz',
            default_value='false',
            description='Run rviz node'),

        Node(
            package='spot_micro_keyboard_command',
            executable='spotMicroKeyboardMove.py',
            name='spot_micro_keyboard_command_node',
            output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('spot_micro_plot'),
                'launch',
                'start_plotting_launch.py')),
            condition=IfCondition(run_plot)),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('spot_micro_rviz'),
                'launch',
                'show_model_launch.py')),
            condition=IfCondition(run_rviz)),
    ])
