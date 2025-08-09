from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    return LaunchDescription([
        
        # Static transforms (since no URDF)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
            name='static_tf_map_to_base'
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser_frame'],
            name='static_tf_base_to_laser'
        ),
        
        # PCD Publisher (your custom node)
        Node(
            package='ply_publisher',  # Replace with your package name
            executable='pcd_publisher',
            name='pcd_publisher',
            output='screen'
        ),
        
        # Convert 3D point cloud to 2D laser scan
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            parameters=[{
                'target_frame': 'laser_frame',
                'transform_tolerance': 0.01,
                'min_height': 0.0,
                'max_height': 2.0,  # Consider all heights for laser scan
                'angle_min': -3.14159,  # Full 360 degrees
                'angle_max': 3.14159,
                'angle_increment': 0.0174533,  # 1 degree
                'scan_time': 0.1,
                'range_min': 0.1,
                'range_max': 100.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            remappings=[
                ('cloud_in', '/obstacle_pointcloud'),
                ('scan', '/scan')
            ]
        ),
        
        # Costmap 2D
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='costmap_2d',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'global_frame': 'map',
                'robot_base_frame': 'base_link',
                'update_frequency': 5.0,
                'publish_frequency': 2.0,
                'width': 100,
                'height': 100,
                'resolution': 0.1,
                'origin_x': -50.0,
                'origin_y': -50.0,
                'plugins': ['obstacle_layer'],
                'obstacle_layer': {
                    'plugin': 'nav2_costmap_2d::ObstacleLayer',
                    'enabled': True,
                    'observation_sources': 'scan',
                    'scan': {
                        'topic': '/scan',
                        'max_obstacle_height': 2.0,
                        'clearing': True,
                        'marking': True,
                        'data_type': 'LaserScan',
                        'raytrace_max_range': 100.0,
                        'raytrace_min_range': 0.0,
                        'obstacle_max_range': 50.0,
                        'obstacle_min_range': 0.0
                    }
                }
            }]
        )
    ])