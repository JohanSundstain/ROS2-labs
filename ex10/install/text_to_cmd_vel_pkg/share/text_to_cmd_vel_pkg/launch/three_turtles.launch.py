from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='text_to_cmd_vel_pkg',
            executable='text_to_cmd_vel',
            name='text_to_cmd_vel'
        )
    ])

