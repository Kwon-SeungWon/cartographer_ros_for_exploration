from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

rviz_config_default =  'ammr.rviz'
def generate_launch_description():

    rviz_config_file = LaunchConfiguration('rviz_config')

    declare_rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value= rviz_config_default,
        description='Name of the RViz config file (must be inside the rviz folder of the package)'
    )

    rviz_config_path = [os.path.join(
        get_package_share_directory('caselab_rviz_plugin'),
        'rviz'
    ), '/', rviz_config_file]

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        declare_rviz_config_arg,
        rviz_node
    ])
