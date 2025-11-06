"""
Launch file for the Robot State Publisher.

This file:
- Processes the Xacro robot description
- Applies configuration arguments (sensors, ros2_control, sim time)
- Publishes the robot state to TF based on the URDF

The URDF is generated at runtime from the Xacro, so changes in arguments
(such as enabling LiDAR or depth camera) reflect directly in the TF tree.
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro

def generate_launch_description():
    
    # ---------------- Launch Arguments ----------------
    sensors_arg = DeclareLaunchArgument(
        'sensors',
        default_value='depth_camera',
        description='Which sensors to enable (depth_camera, lidar_3D)'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Enable or disable ros2_control plugins inside the URDF
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    
    # ---------------- Process URDF/Xacro ----------------
    pkg_path = os.path.join(get_package_share_directory('tizio_bot'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')

    robot_description_config = Command([
        'xacro ', xacro_file,
        ' use_ros2_control:=', use_ros2_control,
        ' sensors:=', LaunchConfiguration('sensors')
    ])


    # ---------------- Robot State Publisher ----------------
    params = {
        'robot_description': robot_description_config,
         'use_sim_time': use_sim_time
    }

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )


    # ---------------- Assemble Launch Description ----------------
    return LaunchDescription([
        sensors_arg,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control if true'),

        node_robot_state_publisher
    ])
