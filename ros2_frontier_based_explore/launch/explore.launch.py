from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    explore_dir = get_package_share_directory('ros2_frontier_based_explore')

    slam_launch_file = os.path.join(explore_dir, 'launch', 'explore_slam.launch.py')
    navigation_launch_file = os.path.join(explore_dir, 'launch', 'explore_nav2_samsung_isaac.launch.py')
    rviz_config_file = os.path.join(explore_dir, 'rviz', 'explore.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_file),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch_file),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),
        Node(
            package='ros2_frontier_based_explore',
            executable='explorer',
            name='explorer_node',
            parameters=[{
                "use_sim_time": use_sim_time,
                "planner_frequency": 0.1,
                "distance_threshold": 100.0, #100m
                "potential_scale": 1.0,
                "gain_scale": 1.0,
                "min_frontier_size": 1.5,
                "orientation_scale": 0.0,
                "progress_timeout": 60,
                "visualize": True,
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])
