import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    package_name = 'tizio_bot' 
    
    # Base NVBlox configuration (always loaded)
    base_config_path = os.path.join(
        get_package_share_directory(package_name),
        'config/nvblox',
        'nvblox_base_real_lidar.yaml'
    )

    # --- REMAPPINGS ---
    remappings = [
        ('pointcloud', '/rslidar_points'),
        ('odom', '/kiss/odometry') 
    ]

    # --- NvBlox Node ---
    nvblox_node = Node(
        package='nvblox_ros',
        executable='nvblox_node',
        name='nvblox_node',
        output='screen',
        parameters=[base_config_path],
        remappings=remappings
    )

    # static_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_tf_base_to_lidar',
    #     arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'rslidar']
    # )

    return LaunchDescription([
        nvblox_node,
        #static_tf
    ])