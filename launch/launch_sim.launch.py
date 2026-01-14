"""
Main launch file for full simulation of Tizio Bot in Gazebo.
This file starts:
- Gazebo with parameters
- Robot State Publisher and controllers
- Joystick teleop
- NVBlox (3D reconstruction)
- Navigation2
- RViz visualization, auto-loaded based on selected sensors
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nvblox_ros_python_utils.nvblox_launch_utils import NvbloxMode


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
            
            # 'range_min': 0.3,       
            # 'range_max': 12.0,            
            
            'scan_time': 0.1,       
            'concurrency_level': 1,
        }],
        remappings=[
            ('cloud_in', '/lidar_points'),
            ('scan', '/scan')
        ]
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
            'sensors': LaunchConfiguration('sensors')
        }.items()
    )


    # ---------------------- Navigation: Nav2 ----------------------
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'nav2.launch.py'
        )]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    
    # ---------------------- Visualization: RViz ----------------------
    def launch_rviz(context, *args, **kwargs):
        sensors_value = LaunchConfiguration('sensors').perform(context)
        if sensors_value == 'depth_camera':
            rviz_file = 'depth_camera.rviz'
        elif sensors_value == 'lidar_3D':
            rviz_file = 'lidar_3D.rviz'
        elif sensors_value == 'lidar':
            rviz_file = 'lidar.rviz'
        elif sensors_value == 'lidar_3D_and_Slam':
            rviz_file = 'lidar_3D_map.rviz'
            

        rviz_config = os.path.join(
            get_package_share_directory(package_name), 'config', rviz_file
        )

        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
        return [rviz_node]

    rviz_launcher = OpaqueFunction(function=launch_rviz)

    # ---------------------- LAUNCHER ----------------------
    return LaunchDescription([
        nvblox_mode_arg,
        sensors_arg,
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

