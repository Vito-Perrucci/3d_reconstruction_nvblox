"""
Main launch file for the full Gazebo simulation of Tizio Bot.

This script orchestrates the complete simulation environment, including:
- Validation: Integrated check for sensor/navigation compatibility.
- Environment: Gazebo simulator with custom physical parameters.
- Robot Core: Robot State Publisher, diff-drive controllers, and joint state broadcasting.
- Control Layer: Joystick teleoperation and Twist Mux for velocity command arbitration.
- 3D Reconstruction: NVBlox integration for real-time volumetric mapping.
- Perception: Pointcloud-to-LaserScan conversion (active if Nav2 is enabled).
- Navigation: Optional Navigation2 stack for autonomous movement.
- Visualization: RViz2 with dynamic configuration loading based on the selected sensor suite and global frame.

Arguments:
    - nvblox_mode: Mapping behavior (static, dynamic, etc.).
    - sensors: Sensor suite selection (depth_camera, lidar_3d, lidar).
    - use_nav2: Boolean to enable/disable the Navigation stack.
    - voxel_size: Resolution of the 3D reconstruction.
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nvblox_ros_python_utils.nvblox_launch_utils import NvbloxMode
from launch.conditions import IfCondition


# ---------------------- Validation Check ----------------------
def validate_setup(context, *args, **kwargs):
    sensors_value = LaunchConfiguration('sensors').perform(context)
    use_nav2_value = LaunchConfiguration('use_nav2').perform(context).lower()

    if sensors_value == 'depth_camera' and use_nav2_value == 'true':
        # Stampiamo l'errore e interrompiamo il processo
        raise RuntimeError(
            "\n\n"
            "*******************************************************************************\n"
            " ERROR: Incompatible configuration detected!\n"
            " Depth camera configuration is not sufficient to make Nav2 works.\n"
            " Please use a LiDAR sensor for Navigation2 or disable Nav2.\n"
            "*******************************************************************************\n"
        )
    return

def generate_launch_description():

    package_name='tizio_bot'


    # ---------------------- Launch Arguments ----------------------
    nvblox_mode_arg = DeclareLaunchArgument(
        'nvblox_mode',
        default_value='static',
        description='Mode for NVBlox (static, people segmentation, people detection, or dynamic)'
    )
    sensors_arg = DeclareLaunchArgument(
        'sensors',
        default_value='depth_camera',
        description='Which sensors to enable (depth_camera, lidar_3d, lidar)'
    )
    use_nav2_arg = DeclareLaunchArgument(
        'use_nav2',
        default_value='false',
        description='Enable launch Nav2'
    )
    voxel_size_arg = DeclareLaunchArgument(
        'voxel_size',
        default_value= '0.1',
        description='size of the voxels for the 3D reconstruction'
    )
    

    validation_check = OpaqueFunction(function=validate_setup)

    # ---------------------- Simulation: Gazebo ----------------------
    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
             )


    # ---------------------- Robot State Publisher ----------------------
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]),
                launch_arguments={
                    'use_sim_time': 'true',
                    'use_ros2_control': 'true',
                    'sensors': LaunchConfiguration('sensors')
                }.items()
    )

    # Spawn robot inside Gazebo from the robot_description topic
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'tizio_bot'],
                        output='screen')


    # ---------------------- Control & Teleoperation ----------------------
    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items())
    
    
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    # In case of use of Nav2 to create the 2D map a 2D-Lidar is needed
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'target_frame': 'laser_frame',
            'transform_tolerance': 0.01,
            
            'min_height': 0.15,        
            'max_height': 0.20,               
            
            'scan_time': 0.1,       
            'concurrency_level': 1,
        }],
        remappings=[
            ('cloud_in', '/lidar_points'),
            ('scan', '/scan')
        ],
        condition=IfCondition(LaunchConfiguration('use_nav2'))
    )


    # twist mux (merging two cmd_vel sources)
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory(package_name),
            'config',
            'twist_mux.yaml'
        )],
        remappings=[
            ('cmd_vel_out', 'diff_cont/cmd_vel_unstamped')
        ]
    )


    # ---------------------- 3D Reconstruction: NVBlox ----------------------
    nvblox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'nvblox.launch.py'
        )]),
        launch_arguments={
            'mode': LaunchConfiguration('nvblox_mode'),
            'sensors': LaunchConfiguration('sensors'),
            'use_nav2': LaunchConfiguration('use_nav2'),
            'voxel_size': LaunchConfiguration('voxel_size'),
            'frame_id': 'base_link',
            'global_frame': 'odom'
        }.items()
    )


    # ---------------------- Navigation: Nav2 ----------------------
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'nav2.launch.py'
        )]),
        launch_arguments={'use_sim_time': 'true'}.items(),
        condition=IfCondition(LaunchConfiguration('use_nav2'))
    )

    
    # ---------------------- Visualization: RViz ----------------------
    def launch_rviz(context, *args, **kwargs):

        sensors_value = LaunchConfiguration('sensors').perform(context)
        use_nav2_value = LaunchConfiguration('use_nav2').perform(context).lower()

        if sensors_value == 'depth_camera':
            rviz_file = 'depth_camera.rviz'
        elif sensors_value == 'lidar_3D':
            rviz_file = 'lidar_3D.rviz'
        elif sensors_value == 'lidar':
            rviz_file = 'lidar.rviz'
        elif sensors_value == 'lidar_3D_and_Slam':
            rviz_file = 'lidar_3D_map.rviz'
            

        rviz_config = os.path.join(
            get_package_share_directory(package_name), 'config','rviz', rviz_file
        )

        fixed_frame = 'map' if use_nav2_value == 'true' else 'odom'

        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config, '-f', fixed_frame],
            output='screen'
        )
        return [rviz_node]

    rviz_launcher = OpaqueFunction(function=launch_rviz)

    # ---------------------- LAUNCHER ----------------------
    return LaunchDescription([
        nvblox_mode_arg,
        sensors_arg,
        use_nav2_arg,
        voxel_size_arg,
        validation_check,
        rsp,
        joystick,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        nvblox,
        twist_mux,
        nav2,
        rviz_launcher,
        pointcloud_to_laserscan_node
    ])

