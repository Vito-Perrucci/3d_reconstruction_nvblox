"""
Launch file for starting the Navigation2 (Nav2) stack with a custom configuration.

This file loads:
- Nav2 bringup_launch.py with simulation time enabled
- Custom navigation parameters (nav2_params.yaml)
- A map (empty or not, but it's needed)

Useful when running navigation in a simulated environment with maps provided manually.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'tizio_bot'

    # Path to Nav2 configuration parameters
    nav2_params = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'nav2_params.yaml'
    )

    # Launch Nav2 bringup
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params,
            'slam': 'False',
            'map': os.path.join(
                get_package_share_directory(package_name),
                'maps',
                'cones_save.yaml'
                # 'face_to_wall_save.yaml'
            )
        }.items()
    )

    return LaunchDescription([
        nav2_launch
    ])
