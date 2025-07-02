#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch arguments
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='/home/kwon/chungnam_tp.yaml',
        description='Full path to the map YAML file'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz'
    )
    
    # Get package directories
    waypoint_pkg_dir = get_package_share_directory('waypoint_node_manager')
    
    # Map server node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map_file'),
            'frame_id': 'map'
        }]
    )
    
    # Waypoint node manager
    waypoint_node_manager = Node(
        package='waypoint_node_manager',
        executable='waypoint_node_manager',
        name='waypoint_node_manager',
        output='screen'
    )
    
    # RViz for visualization (using package's rviz config)
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('waypoint_node_manager'),
        'rviz',
        'node_visualize.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    # Lifecycle manager for nav2 nodes
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server'],
            'bond_timeout': 4.0
        }]
    )
    
    return LaunchDescription([
        map_file_arg,
        use_rviz_arg,
        map_server_node,
        waypoint_node_manager,
        lifecycle_manager,
        rviz_node
    ]) 