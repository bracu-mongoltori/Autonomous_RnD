#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package directories
    depth_smoothing_dir = FindPackageShare('depth_smoothing')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Configuration files
    depth_smoothing_params = PathJoinSubstitution([
        depth_smoothing_dir,
        'config',
        'depth_smoothing_params.yaml'
    ])
    
    # Depth smoothing node only
    depth_smoothing_node = Node(
        package='depth_smoothing',
        executable='depth_smoothing_node',
        name='depth_smoothing_node',
        parameters=[depth_smoothing_params, {
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )
    
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        
        # Nodes
        depth_smoothing_node,
    ])
