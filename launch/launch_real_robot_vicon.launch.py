"""
Main launch file for real-world data processing using VICON Motion Capture.

This script manages a sophisticated pipeline for high-precision 3D reconstruction, 
integrating external tracking and LiDAR data:
- Data Handling: Supports "live" experimentation or "bag" replay.
- Robot Description: Loads the Go1 robot model (URDF/Xacro) for visualization and TF tree.
- Motion Capture Integration: Interfaces with the VICON system to obtain millimetric 
  ground-truth odometry (active in 'live' mode).
- Timestamp Synchronization: A dedicated Python-based node resynchronizes LiDAR 
  pointcloud timestamps with the system clock to ensure TF alignment during live tests.
- Static Transforms: Defines crucial offsets, including the vertical displacement 
  between the VICON marker (cane_cane) and the robot base, and the LiDAR placement.
- 3D Reconstruction: Executes NVBlox in 'real_lidar_vicon' mode, using VICON-accurate 
  poses for high-fidelity volumetric mapping.
- Visualization: RViz2 setup for real-time monitoring of the reconstruction.

Arguments:
    - data_source: 'live' (for laboratory tests with VICON) or 'bag' (for offline replay).
    - voxel_size: Resolution of the 3D reconstruction grid.
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, ExecuteProcess, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression

def generate_launch_description():

    package_name = 'tizio_bot'
    package_name_description = 'go1_description'
    package_name_lidar = 'rslidar_sdk'
    package_vicon = 'vicon_receiver'

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
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name_description), 'launch', 'load_go1.launch.py'
        )]),
        launch_arguments={
            'use_jsp': 'jsp',
            'use_rviz': 'false',   
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

    # ---------------------- VICON SETUP ----------------------
    vicon_receiver_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_vicon), 'launch', 'client.launch.py'
        )]),
        launch_arguments={
            'hostname': '192.168.50.56',
            'topic_namespace': 'vicon',
            'buffer_size': '200',
            'world_frame': 'map',
            'vicon_frame': 'vicon',
            'map_xyz': '[0.0, 0.0, 0.0]',
            'map_rpy': '[0.0, 0.0, 0.0]',
            'map_rpy_in_degrees': 'false'
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('data_source'), "' == 'live'"])
        )
    )

    # ---------------------- TIMESTAMP SYNCHRONIZER NODE ----------------------
    synchronizer_node = GroupAction(
        actions=[
            ExecuteProcess(
                cmd=['python3', '-c', 
                    'import rclpy; from sensor_msgs.msg import PointCloud2; rclpy.init(); '
                    'node = rclpy.create_node("sync_node"); '
                    'pub = node.create_publisher(PointCloud2, "/rslidar_points_fixed", 10); '
                    'sub = node.create_subscription(PointCloud2, "/rslidar_points", '
                    'lambda msg: (setattr(msg.header, "stamp", node.get_clock().now().to_msg()), pub.publish(msg)), 10); '
                    'rclpy.spin(node)'
                ],
                output='screen',
                name='rslidar_timestamp_sync'
            )
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('data_source'), "' == 'live'"])
        )
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

    static_tf_cane_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_cane_to_base',
        arguments=['0', '0', '-0.60', '0', '0', '0', 'cane_cane', 'base'],
        parameters=[{'use_sim_time': False}]
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
            'sensors': 'real_lidar_vicon',
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
        vicon_receiver_node, 
        synchronizer_node,
        nvblox,
        rviz_launcher,
        static_tf_cane_to_base,
        static_tf_truck_to_lidar
    ])
