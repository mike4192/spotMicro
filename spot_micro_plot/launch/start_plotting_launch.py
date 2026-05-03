from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='spot_micro_plot',
            executable='spotMicroPlot.py',
            name='spot_micro_plot_node',
            output='screen'),
    ])
