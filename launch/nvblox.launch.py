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
from nvblox_ros_python_utils.nvblox_launch_utils import NvbloxMode

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

    mode_str = LaunchConfiguration('mode').perform(context)
    sensors_str = LaunchConfiguration('sensors').perform(context)
    parameters = get_parameters(mode_str)

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
    
    # Choose remapping based on sensor type
    if sensors_str == 'depth_camera':
        chosen_params = remappings_depth
    elif sensors_str == 'lidar_3D':
        chosen_params = remapping_lidar_3d

    # NVBlox node
    nvblox_node = Node(
        package='nvblox_ros',
        executable='nvblox_node',
        name='nvblox_node',
        output='screen',
        parameters=parameters,
        remappings=chosen_params
    )

    return  [nvblox_node]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
