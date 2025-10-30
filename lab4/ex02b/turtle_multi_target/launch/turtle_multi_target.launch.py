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

		spawn_turtle3 = ExecuteProcess(
			cmd=[
				'ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn',
				'{x: 1.0, y: 1.0, theta: 0.0, name: "turtle3"}'
			],
			output='screen'
		)

		# 4. turtle1 спавнится автоматически, но мы хотим её телепортировать
		teleport_turtle1 = ExecuteProcess(
			cmd=[
				'ros2', 'service', 'call', '/turtle1/teleport_absolute', 'turtlesim/srv/TeleportAbsolute',
				'{x: 10.0, y: 10.0, theta: 0.0}'
			],
			output='screen'
		)

		# 5. Другие узлы
		switcher = Node(
			package='turtle_multi_target',
			executable='target_switcher',
			name='target_switcher',
			output='screen'
		)

		controller = Node(
			package='turtle_multi_target',
			executable='turtle_controller',
			name='turtle_controller',
			output='screen'
		)

		rviz_node = Node(
			package='rviz2',
			executable='rviz2',
			name='rviz2',
			arguments=['-d', '~/ros2_ws/src/turtle_multi_target/rviz/rviz.rviz'],
			output='screen'
		)

		

		spawn_turtle2_after_turtlesim = RegisterEventHandler(
			event_handler=OnProcessExit(
				target_action=turtlesim,
				on_exit=[spawn_turtle2]
			)
		)

		spawn_turtle3_after_turtlesim = RegisterEventHandler(
			event_handler=OnProcessExit(
				target_action=turtlesim,
				on_exit=[spawn_turtle3]
			)
		)


		teleport_turtle1_after_turtlesim = RegisterEventHandler(
			event_handler=OnProcessExit(
				target_action=spawn_turtle3,
				on_exit=[teleport_turtle1]
			)
		)
		return LaunchDescription([
			turtlesim,
			spawn_turtle2,
			spawn_turtle3,
			teleport_turtle1_after_turtlesim,  
			spawn_turtle2_after_turtlesim,
			spawn_turtle3_after_turtlesim,
			switcher,
			controller,
			rviz_node
		])