"""
Launch file for joystick teleoperation.

This starts:
- joy_node: reads joystick input from /dev/input/js*
- teleop_twist_joy: converts joystick inputs into geometry_msgs/Twist commands

The /cmd_vel output is remapped to /cmd_vel_joy so that Twist Mux or Nav2
can merge navigation commands properly without conflicts.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Shared joystick configuration file
    joy_params = os.path.join(get_package_share_directory('tizio_bot'),'config','joystick.yaml')

    # Node that reads raw joystick events
    joy_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params, {'use_sim_time': use_sim_time}],
         )

    # Converts joystick inputs to cmd_vel; output remapped for Twist Mux
    teleop_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joy_params, {'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel','/cmd_vel_joy')]
         )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        joy_node,
        teleop_node,
    ])