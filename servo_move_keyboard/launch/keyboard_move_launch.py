from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    run_i2c_pwmboard = LaunchConfiguration('run_i2c_pwmboard')

    return LaunchDescription([
        DeclareLaunchArgument(
            'run_i2c_pwmboard',
            default_value='false',
            description='Also run i2c_pwmboard node'),

        Node(
            package='servo_move_keyboard',
            executable='servoMoveKeyboard.py',
            name='servo_move_keyboard_node',
            output='screen'),

        Node(
            package='i2cpwm_board',
            executable='i2cpwm_board_node',
            name='i2cpwm_board_node',
            output='screen',
            condition=IfCondition(run_i2c_pwmboard)),
    ])
