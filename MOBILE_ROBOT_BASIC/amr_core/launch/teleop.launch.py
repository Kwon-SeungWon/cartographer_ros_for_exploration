from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='amr_core',  
            executable='teleop_keyboard',
            name='teleop_keyboard',
            output='screen',
            parameters=[
                {
                    'topic_name': 'manual_vel',
                    'speed': 0.25,
                    'turn': 0.25,
                    'control_hz': 10.0
                }
            ]
        ),
    ])
