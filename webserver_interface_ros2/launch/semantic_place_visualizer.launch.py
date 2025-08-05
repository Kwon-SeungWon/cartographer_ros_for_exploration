#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 패키지 디렉토리 가져오기
    pkg_share = get_package_share_directory('webserver_interface_ros2')
    
    # Launch 인자들
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Semantic Place Visualizer 노드
    semantic_visualizer_node = Node(
        package='webserver_interface_ros2',
        executable='semantic_place_visualizer',
        name='semantic_place_visualizer',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('semantic_places', '/semantic_places'),
        ]
    )
    
    # Map Server 노드
    map_yaml_path = '/home/caselab/catkin_ws/src/Wheelchair_Robot_ROS2_Project/data/map_data/maps/geumjeong_show.yaml'
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml_path,
            'use_sim_time': use_sim_time,
        }]
    )
    
    # Lifecycle Manager 노드 (map_server 자동 활성화)
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server'],
            'use_sim_time': use_sim_time,
        }]
    )
    
    # RViz2 실행 (설정 파일과 함께)
    rviz_config_path = os.path.join(pkg_share, 'config', 'semantic_places.rviz')
    rviz_node = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config_path],
        output='screen'
    )
    
    return LaunchDescription([
        # Launch 인자 선언
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        # 노드 추가
        map_server_node,
        lifecycle_manager_node,
        semantic_visualizer_node,
        rviz_node,
    ]) 