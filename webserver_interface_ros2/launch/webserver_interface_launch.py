from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pt_gunsol',
            executable='subscriber_service_all',
            name='subscriber_service_all',
            output='screen'
        ),

        Node(
            package='pt_gunsol_instant_actions',
            executable='publisher_service_all',
            name='publisher_service_all',
            output='screen'
        ),

        Node(
            package='webserver_interface_ros2',
            executable='ros_handler',
            name='ros_handler',
            output='screen'
        ),
    ])