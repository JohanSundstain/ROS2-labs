# launch/turtles_with_carrot_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    radius_arg = DeclareLaunchArgument('radius', default_value='1.0',
                                       description='distance from turtle1 to carrot')
    direction_arg = DeclareLaunchArgument('direction_of_rotation', default_value='1',
                                          description='1 for clockwise, -1 for counter-clockwise')

    turtlesim = Node(package='turtlesim', executable='turtlesim_node', name='sim')

    manager = Node(
        package='turtles_twowithcarrot',
        executable='turtles_manager',
        name='turtles_manager',
        parameters=[
            {'radius': LaunchConfiguration('radius')},
            {'direction_of_rotation': LaunchConfiguration('direction_of_rotation')}
        ],
        output='screen'
    )

    teleop = ExecuteProcess(
        cmd=['ros2', 'run', 'turtlesim', 'turtle_teleop_key'],
        shell=False,
        name='teleop'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', [LaunchConfiguration('rviz_config')]],
        # we will pass rviz_config param via env or adjust below; simpler: start rviz without config
    )

    # If you want to provide rviz config from package rviz/carrot.rviz:
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', 'share/turtles_twowithcarrot/rviz/carrot.rviz'],
        output='screen'
    )

    return LaunchDescription([
        radius_arg,
        direction_arg,
        turtlesim,
        manager,
        # teleop: user might want to run teleop in separate terminal; including it here will block the launch
        # so we provide it but comment: user can run in separate terminal if desired.
        rviz_node
    ])
