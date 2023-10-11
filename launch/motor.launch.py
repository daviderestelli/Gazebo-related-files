from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    config = os.path.join(get_package_share_directory('motor_control'),'config','params.yaml')

    # Driving Motors
    motor1 = Node(
		package='motor_control',
		executable='run_motor',
		name='motor1',
		parameters=[config],
	)
    motor2 = Node(
		package='motor_control',
		executable='run_motor',
		name='motor2',
		parameters=[config],
	)
    motor3 = Node(
		package='motor_control',
		executable='run_motor',
		name='motor3',
		parameters=[config],
	)
    motor4 = Node(
		package='motor_control',
		executable='run_motor',
		name='motor4',
		parameters=[config],
    )

    # Steering Motors
    steer = Node(
        package='motor_control',
        executable='steer_motor',
        name='steer',
        parameters=[config],
	)
    # steer2 = Node(
    #     package='motor_control',
    #     executable='steer_motor',
    #     name='steer1',
    #     parameters=[config],
	# )
    # steer3 = Node(
    #     package='motor_control',
    #     executable='steer_motor',
    #     name='steer1',
    #     parameters=[config],
	# )
    # steer4 = Node(
    #     package='motor_control',
    #     executable='steer_motor',
    #     name='steer1',
    #     parameters=[config],
	# )
    

    return LaunchDescription([
        motor1,
        motor2,
        motor3,
        motor4,
        steer,
    ])
