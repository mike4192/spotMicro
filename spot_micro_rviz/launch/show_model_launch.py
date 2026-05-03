import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    model = LaunchConfiguration('model')
    rvizconfig = LaunchConfiguration('rvizconfig')

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=os.path.join(
                get_package_share_directory('spot_micro_rviz'),
                'urdf',
                'spot_micro.urdf.xacro'),
            description='Path to robot URDF file'),
        DeclareLaunchArgument(
            'rvizconfig',
            default_value=os.path.join(
                get_package_share_directory('spot_micro_rviz'),
                'rviz',
                'spot_micro.rviz'),
            description='Path to RViz config file'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', model])}]),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', rvizconfig],
            output='screen'),
    ])
