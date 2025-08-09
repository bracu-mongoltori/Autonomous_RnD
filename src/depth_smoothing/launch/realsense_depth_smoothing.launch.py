#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    depth_smoothing_dir = FindPackageShare('depth_smoothing')
    realsense2_camera_dir = FindPackageShare('realsense2_camera')
    depthimage_to_laserscan_dir = FindPackageShare('depthimage_to_laserscan')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    enable_depth_arg = DeclareLaunchArgument(
        'enable_depth',
        default_value='true',
        description='Enable depth camera'
    )
    
    enable_color_arg = DeclareLaunchArgument(
        'enable_color',
        default_value='true', 
        description='Enable color camera'
    )
    
    depth_width_arg = DeclareLaunchArgument(
        'depth_width',
        default_value='640',
        description='Depth image width'
    )
    
    depth_height_arg = DeclareLaunchArgument(
        'depth_height',
        default_value='480',
        description='Depth image height'
    )
    
    depth_fps_arg = DeclareLaunchArgument(
        'depth_fps',
        default_value='30',
        description='Depth image FPS'
    )
    
    # Configuration files
    depth_smoothing_params = PathJoinSubstitution([
        depth_smoothing_dir,
        'config',
        'depth_smoothing_params.yaml'
    ])
    
    depth_to_laserscan_params = PathJoinSubstitution([
        depth_smoothing_dir,
        'config', 
        'depth_to_laserscan_params.yaml'
    ])
    
    # RealSense camera launch
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                realsense2_camera_dir,
                'launch',
                'rs_launch.py'
            ])
        ]),
        launch_arguments={
            'enable_depth': LaunchConfiguration('enable_depth'),
            'enable_color': LaunchConfiguration('enable_color'),
            'depth_module.profile': [LaunchConfiguration('depth_width'), 'x', 
                                   LaunchConfiguration('depth_height'), 'x',
                                   LaunchConfiguration('depth_fps')],
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'enable_gyro': 'false',
            'enable_accel': 'false',
        }.items()
    )
    
    # Depth smoothing node
    depth_smoothing_node = Node(
        package='depth_smoothing',
        executable='depth_smoothing_node',
        name='depth_smoothing_node',
        parameters=[depth_smoothing_params, {
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        remappings=[
            ('/camera/depth/image_rect_raw', '/camera/camera/depth/image_rect_raw'),
            ('/camera/depth/camera_info', '/camera/camera/depth/camera_info'),
        ],
        output='screen'
    )
    
    # Depth to laserscan node
    depth_to_laserscan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        parameters=[depth_to_laserscan_params, {
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        remappings=[
            ('depth', '/camera/depth/smoothed_image'),
            ('depth_camera_info', '/camera/camera/depth/camera_info'),
            ('scan', '/scan'),
        ],
        output='screen'
    )
    
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        enable_depth_arg,
        enable_color_arg,
        depth_width_arg,
        depth_height_arg,
        depth_fps_arg,
        
        # Nodes
        realsense_launch,
        depth_smoothing_node,
        depth_to_laserscan_node,
    ])
