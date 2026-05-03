import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    run_standalone = LaunchConfiguration('run_standalone')
    debug_mode = LaunchConfiguration('debug_mode')
    run_lcd = LaunchConfiguration('run_lcd')

    config = os.path.join(
        get_package_share_directory('spot_micro_motion_cmd'),
        'config',
        'spot_micro_motion_cmd.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'run_standalone',
            default_value='false',
            description='Run without i2c_pwmboard'),
        DeclareLaunchArgument(
            'debug_mode',
            default_value='false',
            description='Enable debug mode'),
        DeclareLaunchArgument(
            'run_lcd',
            default_value='false',
            description='Run lcd monitor node'),

        Node(
            package='i2cpwm_board',
            executable='i2cpwm_board_node',
            name='i2cpwm_board_node',
            output='screen',
            condition=UnlessCondition(run_standalone)),

        Node(
            package='spot_micro_motion_cmd',
            executable='spot_micro_motion_cmd_node',
            name='spot_micro_motion_cmd_node',
            output='screen',
            parameters=[config]),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('lcd_monitor'),
                'launch',
                'lcd_monitor_launch.py')),
            condition=IfCondition(run_lcd)),
    ])
