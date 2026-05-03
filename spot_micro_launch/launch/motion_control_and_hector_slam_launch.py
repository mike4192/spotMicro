import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    run_post_proc = LaunchConfiguration('run_post_proc')
    geotiff_map_file_path = LaunchConfiguration('geotiff_map_file_path')

    return LaunchDescription([
        DeclareLaunchArgument(
            'run_post_proc',
            default_value='false',
            description='Run only hector_mapping in post processing mode'),
        DeclareLaunchArgument(
            'geotiff_map_file_path',
            default_value='',
            description='Path for geotiff map output'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('spot_micro_motion_cmd'),
                'launch',
                'motion_cmd_launch.py')),
            condition=UnlessCondition(run_post_proc)),

        # RPLIDAR node - parameters may need adjustment based on your ROS2 rplidar_ros version
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidarNode',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'lidar_link',
                'inverted': False,
                'angle_compensate': True,
            }],
            condition=UnlessCondition(run_post_proc)),

        # Hector mapping - may require external ROS2 port of hector_slam
        Node(
            package='hector_mapping',
            executable='hector_mapping',
            name='hector_mapping',
            output='screen',
            parameters=[{
                'map_frame': 'map',
                'base_frame': 'base_footprint',
                'odom_frame': 'odom',
                'use_tf_scan_transformation': True,
                'use_tf_pose_start_estimate': False,
                'pub_map_odom_transform': True,
                'map_resolution': 0.050,
                'map_size': 2048,
                'map_start_x': 0.5,
                'map_start_y': 0.5,
                'map_multi_res_levels': 2,
                'update_factor_free': 0.4,
                'update_factor_occupied': 0.9,
                'map_update_distance_thresh': 0.4,
                'map_update_angle_thresh': 0.06,
                'laser_z_min_value': -1.0,
                'laser_z_max_value': 1.0,
                'advertise_map_service': True,
                'scan_subscriber_queue_size': 5,
                'scan_topic': 'scan',
                'tf_map_scanmatch_transform_frame_name': 'scanmatcher_frame',
            }]),
    ])
