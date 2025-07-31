import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    sick_scan_pkg_prefix = get_package_share_directory('amr_core')

    front_launch_file_path = os.path.join(sick_scan_pkg_prefix, 'launch/sick_picoscan_front.launch')
    rear_launch_file_path = os.path.join(sick_scan_pkg_prefix, 'launch/sick_picoscan_rear.launch')

    front_node_arguments = [front_launch_file_path]
    rear_node_arguments = [rear_launch_file_path]

    # Add any command-line style arguments passed into both
    for arg in sys.argv:
        if len(arg.split(":=")) == 2:
            front_node_arguments.append(arg)
            rear_node_arguments.append(arg)

    ROS_DISTRO = os.environ.get('ROS_DISTRO')
    use_node_executable = ROS_DISTRO[0] <= "e"  # For eloquent and earlier

    # Front LiDAR node
    front_node = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        output='screen',
        arguments=front_node_arguments
    )

    # Rear LiDAR node
    rear_node = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        output='screen',
        arguments=rear_node_arguments
    )

    ld.add_action(front_node)
    ld.add_action(rear_node)

    return ld
