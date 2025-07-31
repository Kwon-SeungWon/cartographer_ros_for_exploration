import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    amr_core_share_dir = get_package_share_directory('amr_core')

    parameter_file = LaunchConfiguration('params_file')

    params_declare = DeclareLaunchArgument(
        'params_file',
        # default_value=os.path.join(amr_core_share_dir, 'param', 'laser_base_exclude_filter.yaml'),
        default_value=os.path.join(amr_core_share_dir, 'param', 'laser_filter.yaml'),
        description='Path to the ROS2 parameters file to use.'
    )

    return LaunchDescription([
        params_declare,
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            parameters=[parameter_file],
        )
    ])
