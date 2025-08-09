import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    pkg_name = 'auto_robot'

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation time'
    )

    # Robot description + TF
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(pkg_name), 'launch', 'rsp.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # Gazebo (optional if not already running via another launch)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('start_gazebo', default='true'))
    )

    declare_start_gazebo = DeclareLaunchArgument(
        'start_gazebo', default_value='true', description='Start Gazebo server & client'
    )

    # Spawn robot if using Gazebo in this launch
    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py', output='screen',
        arguments=['-topic', 'robot_description', '-entity', 'auto_car']
    )

    # Depthimage to LaserScan
    depth_to_scan = Node(
        package='depthimage_to_laserscan', executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan', output='screen',
        parameters=[
            os.path.join(get_package_share_directory(pkg_name), 'config', 'depthimage_to_laserscan.yaml')
        ],
        remappings=[
            # Gazebo depth camera topics (namespace: camera, camera_name: depth)
            ('depth', '/camera/depth/image_raw'),
            ('depth_camera_info', '/camera/depth/camera_info'),
            ('scan', '/scan')
        ]
    )

    # SLAM Toolbox in mapping mode
    slam_toolbox = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox', output='screen',
        parameters=[
            os.path.join(get_package_share_directory(pkg_name), 'config', 'slam_toolbox_depth.yaml'),
            {'use_sim_time': use_sim_time}
        ]
    )

    return LaunchDescription([
        declare_sim_time,
        declare_start_gazebo,
        rsp,
        gazebo,
        spawn_entity,
        depth_to_scan,
        slam_toolbox,
    ])
