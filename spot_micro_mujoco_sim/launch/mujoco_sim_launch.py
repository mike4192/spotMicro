import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("spot_micro_mujoco_sim")
    default_model = os.path.join(pkg_share, "models", "spot_micro_sim.xml")

    # Process xacro to URDF for robot_state_publisher
    xacro_path = os.path.join(
        get_package_share_directory("spot_micro_rviz"),
        "urdf", "spot_micro.urdf.xacro"
    )
    try:
        urdf_xml = subprocess.check_output(["xacro", xacro_path], text=True)
    except Exception:
        urdf_xml = "<robot name=\"spot_micro_rviz\"/>"

    rviz_config = os.path.join(pkg_share, "config", "spot_micro.rviz")
    has_rviz_config = os.path.exists(rviz_config)

    return LaunchDescription([
        DeclareLaunchArgument(
            "model_path",
            default_value=default_model,
            description="Path to MuJoCo MJCF model file",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Launch RViz2 for visualization",
        ),

        # MuJoCo simulation bridge
        Node(
            package="spot_micro_mujoco_sim",
            executable="mujoco_sim_node",
            name="mujoco_sim",
            output="screen",
            parameters=[{
                "model_path": LaunchConfiguration("model_path"),
                "sim_rate_hz": 500.0,
                "publish_rate_hz": 50.0,
                "initial_body_z": 0.25,
            }],
        ),

        # Robot state publisher (publishes TF from joint_states + URDF)
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": urdf_xml}],
        ),

        # RViz2
        Node(
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config] if has_rviz_config else [],
        ),
    ])
