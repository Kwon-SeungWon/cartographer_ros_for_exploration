import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # amr_navigation 패키지 디렉토리 경로를 가져옵니다.
    amr_navigation_share_dir = get_package_share_directory('amr_navigation')

    # rviz 설정 파일 경로를 Launch 파라미터로 선언합니다.
    rviz_file = LaunchConfiguration('rvizs_file')

    # rvizs_file 파라미터를 선언하여, 기본값을 설정합니다.
    rviz_declare = DeclareLaunchArgument(
        'rvizs_file',
        default_value=os.path.join(amr_navigation_share_dir, 'rviz', 'navigation.rviz'),
        description='Path to the ROS2 rviz file to use.'
    )

    # RViz 노드 실행
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': False}],  # 'false' 대신 False로 수정
        arguments=['-d', rviz_file]  # RViz 설정 파일 경로를 -d 옵션으로 전달
    )

    # LaunchDescription 리턴
    return LaunchDescription([
        rviz_declare,  # rvizs_file 파라미터를 선언
        rviz_node,     # RViz 노드 실행
    ])