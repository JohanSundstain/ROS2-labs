from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
		# 1. Запускаем turtlesim
		turtlesim = Node(
			package='turtlesim',
			executable='turtlesim_node',
			name='sim'
		)

		spawn_turtle2 = ExecuteProcess(
			cmd=[
				'ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn',
				'{x: 5.5, y: 5.5, theta: 0.0, name: "turtle2"}'
			],
			output='screen'
		)


		time_travel = Node(
			package='time_travel',
			executable='time_travel',
			name='time_travel',
			output='screen',
			parameters=[{'delay': 1.0}]   
		)

		rviz_node = Node(
			package='rviz2',
			executable='rviz2',
			name='rviz2',
			arguments=['-d', '~/ros2_ws/src/time_travel/rviz/rviz.rviz'],
			output='screen'
		)

		return LaunchDescription([
			turtlesim,
			spawn_turtle2,
			time_travel,
			rviz_node
		])