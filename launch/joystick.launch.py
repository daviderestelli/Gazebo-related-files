from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    joy_params = os.path.join(get_package_share_directory('motor_control'),'config','joystick.yaml')

    joy_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params],
         )

    teleop_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joy_params],
         )

    # midlayer_node = Node(
    #         package='runmot',
    #         executable='midlayer',
    #         name='midlayer',
    #         parameters=[joy_params],
    #      )     

    # publisher_node = Node(
    #         package='runmot',
    #         executable='publisher',
    #         name='publisher',
    #         parameters=[joy_params],
    #      )     

    return LaunchDescription([
        joy_node,
        teleop_node,
        # midlayer_node,
        # publisher_node  
    ])