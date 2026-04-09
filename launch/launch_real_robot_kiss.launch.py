"""
Main launch file for real-world data processing and 3D reconstruction.

This script manages the pipeline for replaying bag data or processing live 
sensor streams from the robot, focusing on LiDAR odometry and volumetric mapping:
- Data Handling: Supports both "live" LiDAR SDK input and "bag" data playback via arguments.
- Robot Description: Loads the Go1 quadruped robot model (URDF/Xacro) via RSP.
- Odometry: Integrates KISS-ICP for robust LiDAR-based odometry and TF generation.
- Static Transforms: Manages essential coordinate frames (map -> odom and trunk -> rslidar) 
  to bridge the gap between the robot base and the sensors.
- 3D Reconstruction: Executes NVBlox in 'real_lidar_kiss' mode for high-fidelity 
  mapping using the LiDAR pointcloud.
- Visualization: Launches RViz2 with a pre-configured profile for real robot monitoring.

Arguments:
    - data_source: Switch between 'live' (real-time SDK) and 'bag' (offline replay).
    - voxel_size: Resolution of the NVBlox 3D reconstruction grid.
"""


import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression

def generate_launch_description():

    package_name = 'tizio_bot'
    package_name_description = 'go1_description'
    package_name_lidar = 'rslidar_sdk'
    package_name_kiss = 'kiss_icp'

    # ---------------------- Launch Arguments ----------------------
    data_source_arg = DeclareLaunchArgument(
        'data_source',
        default_value='bag',
        description='Source of LiDAR data: "bag" or "live"'
    )
    voxel_size_arg = DeclareLaunchArgument(
        'voxel_size',
        default_value= '0.1',
        description='size of the voxels for the 3D reconstruction'
    )


    # ---------------------- Robot State Publisher (rsp) ----------------------
    # Includiamo il file di lancio del pacchetto go1_description
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name_description), 'launch', 'load_go1.launch.py'
        )]),
        launch_arguments={
            'use_jsp': 'jsp',      # (jsp, gui or none)
            'use_rviz': 'false',   
            'fixed_frame': 'map'
        }.items()
    )

    # ---------------------- LiDAR sdk Node ----------------------
    lidar_config_path = os.path.join(
        get_package_share_directory(package_name_lidar), 'config', 'config.yaml'
    )

    rslidar_node = Node(
        package='rslidar_sdk',
        executable='rslidar_sdk_node',
        name='rslidar_sdk_node',
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('data_source'), "' == 'live'"])
        )
    )

    # ---------------------- KISS-ICP Odometry ----------------------
    pointcloud_topic = "/rslidar_points" 
    base_frame = "base"
    lidar_odom_frame = "odom" 

    kiss_icp_node = Node(
        package=package_name_kiss,
        executable="kiss_icp_node",
        name="kiss_icp_node",
        output="screen",
        remappings=[
            ("pointcloud_topic", pointcloud_topic),
        ],
        parameters=[
            {
                "base_frame": base_frame,
                "lidar_odom_frame": lidar_odom_frame,
                "publish_odom_tf": True,
                "publish_odom_tf": True,
                "invert_odom_tf": True,
                "publish_debug_clouds": True,
                "use_sim_time": True,
                "position_covariance": 0.1,
                "orientation_covariance": 0.1,
            },
            os.path.join(get_package_share_directory(package_name_kiss), "config", "config.yaml")
        ]
    )

    # ---------------------- Visualization: RViz ----------------------
    def launch_rviz(context, *args, **kwargs):
        rviz_config = os.path.join(
            get_package_share_directory(package_name), 'config', 'rviz', 'real_robot.rviz'
        )
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config, '-f', "map"],
            output='screen'
        )
        return [rviz_node]

    rviz_launcher = OpaqueFunction(function=launch_rviz)

    # ---------------------- Static Transforms ----------------------
    static_tf_map_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'map'],
        parameters=[{'use_sim_time': True}]
    )

    static_tf_truck_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_truck_to_lidar',
        arguments=['0', '0', '0.12', '0', '0', '0', 'trunk', 'rslidar'],
        parameters=[{'use_sim_time': False}]
    )

    # ---------------------- 3D Reconstruction: NVBlox ----------------------
    nvblox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'nvblox.launch.py'
        )]),
        launch_arguments={
            'voxel_size': LaunchConfiguration('voxel_size'),
            'mode': 'static',
            'sensors': 'real_lidar_kiss',
            'use_nav2': 'false',
            'frame_id': 'rslidar',
            'global_frame': 'map'
        }.items()
    )

    # ---------------------- LAUNCHER ----------------------
    return LaunchDescription([
        data_source_arg,
        voxel_size_arg,
        rsp,
        rslidar_node,
        kiss_icp_node,
        nvblox,
        rviz_launcher,
        static_tf_map_to_base,
        static_tf_truck_to_lidar
    ])