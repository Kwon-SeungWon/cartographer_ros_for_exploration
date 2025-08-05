from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('amr_serial')
    param_file = os.path.join(pkg_share, 'param', 'serial.yaml')

    return LaunchDescription([
        Node(
            package='amr_serial',
            executable='serial_all.py',
            name='serial_node',
            output='screen',
            parameters=[param_file],
        )
    ]) 