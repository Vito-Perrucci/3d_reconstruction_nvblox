"""
Launch file for NVBlox mapping node.

Loads NVBlox with a base configuration plus optional specializations
(people detection, people segmentation, or dynamic reconstruction).

The remappings depend on the active sensor:
- depth camera
- 3D LiDAR

Note:
- Some NVBlox modes (people segmentation, detection, dynamic) are incompatible with LiDAR.
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def get_parameters(mode):
    """Return the list of parameter files for NVBlox based on the selected mode."""

    package_name = 'tizio_bot'

    # Base NVBlox configuration (always loaded)
    base_params = os.path.join(
        get_package_share_directory(package_name),
        'config/nvblox',
        'nvblox_base.yaml'
    )
    # People detection configuration
    detection_params = os.path.join(
        get_package_share_directory(package_name),
        'config/nvblox/specializations',
        'nvblox_detection.yaml'
    )
    # Dynamics configuration
    dynamics_params = os.path.join(
        get_package_share_directory(package_name),
        'config/nvblox/specializations',
        'nvblox_dynamics.yaml'
    )
    # People segmentation configuration
    segmentation_params = os.path.join(
        get_package_share_directory(package_name),
        'config/nvblox/specializations',
        'nvblox_segmentation.yaml'
    )
    
    # Select specialization parameters
    if mode == 'static':
        mode_params = {}
    elif mode == 'people_segmentation':
        mode_params = segmentation_params
        assert not use_lidar, 'Can not run lidar with people segmentation mode.'
    elif mode == 'people_detection':
        mode_params = detection_params
        assert not use_lidar, 'Can not run lidar with people detection mode.'
    elif mode == 'dynamic':
        mode_params = dynamics_params
        assert not use_lidar, 'Can not run lidar with dynamic mode.'
    else:
        raise Exception(f'Mode {mode} not implemented for nvblox.')


    parameters = []
    parameters.append(base_params)
    parameters.append(mode_params)

    return parameters
    

def launch_setup(context, *args, **kwargs):
    """Create the NVBlox node with parameters and remappings based on mode and sensors."""

    ## ARGUMENTS

    # Organizational arguments
    mode_str = LaunchConfiguration('mode').perform(context)
    sensors_str = LaunchConfiguration('sensors').perform(context)
    use_nav2_str = LaunchConfiguration('use_nav2').perform(context)

    # Basic parameters
    parameters = get_parameters(mode_str)

    # Additional parameters
    voxel_size_value = LaunchConfiguration('voxel_size').perform(context)
    frame_id_str = LaunchConfiguration('frame_id').perform(context)
    global_frame_str = LaunchConfiguration('global_frame').perform(context)

    parameters.append({
        'voxel_size': float(voxel_size_value), 
        'global_pose': frame_id_str, 
        'global_frame': global_frame_str, 
        'map_clearing_frame_id': frame_id_str,
        'esdf_slice_bounds_visualization_attachment_frame_id': frame_id_str,
        'workspace_height_bounds_visualization_attachment_frame_id': frame_id_str
    })

    # nav2 Changing for simulation
    if use_nav2_str == 'true':
        parameters.append({'global_frame': map})


    ## REMAPPING

    # Remapping for 3DLiDAR-based mapping
    remapping_lidar_3d = [
        ('pointcloud', '/lidar_points'),
        ('camera_0/color/image', '/camera/image_raw'),
        ('camera_0/color/camera_info', '/camera/camera_info'),
        ('odom', '/odom'),
    ]
    # Remapping for depth-camera-based mapping
    remappings_depth=[
        ('camera_0/depth/image', '/camera/depth/image_raw'),
        ('camera_0/depth/camera_info', '/camera/depth/camera_info'),
        ('camera_0/color/image', '/camera/image_raw'),
        ('camera_0/color/camera_info', '/camera/camera_info'),
        ('odom', '/odom'),
    ]
    # Remapping for real lidar and kiss-icp
    remappings_kiss = [
        ('pointcloud', '/rslidar_points'),
        ('odom', '/kiss/odometry') 
    ]

    # Remapping for real lidar and Vicon
    remappings_vicon = [
        ('pointcloud', '/rslidar_points_fixed'),
        ('odom', '/vicon/cane/cane') 
    ]

    
    # Choose remapping based on sensor type
    if sensors_str == 'depth_camera':
        chosen_remappings = remappings_depth
    elif sensors_str == 'lidar_3D' or sensors_str == 'lidar_3D_and_Slam':
        chosen_remappings = remapping_lidar_3d
    elif sensors_str == 'real_lidar_kiss':
        chosen_remappings = remappings_kiss
    elif sensors_str == 'real_lidar_vicon':
        chosen_remappings = remappings_vicon


    ## NVBlox NODE

    nvblox_node = Node(
        package='nvblox_ros',
        executable='nvblox_node',
        name='nvblox_node',
        output='screen',
        parameters=parameters,
        remappings=chosen_remappings,
        arguments=['--ros-args', '--log-level', 'warn']
    )

    return  [nvblox_node]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
