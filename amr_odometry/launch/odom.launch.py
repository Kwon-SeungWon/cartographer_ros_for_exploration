import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def check_param_file(context, *args, **kwargs):
    param_path = LaunchConfiguration('params_file').perform(context)
    if not os.path.isfile(param_path):
        raise FileNotFoundError(f"[Launch] Parameter file not found: {param_path}")
    return []

def generate_launch_description():
    # Get the package share directory
    amr_odometry_share_dir = get_package_share_directory('amr_odometry')

    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value='src/MOBILE_ROBOT_BASIC/amr_odometry/param/param.yaml',
        description='Relative path to the unified parameter YAML file'
    )

    param_file = LaunchConfiguration('params_file')


    return LaunchDescription([
        declare_params,
        OpaqueFunction(function=check_param_file),  # (optional) path check
        Node(
            package='amr_odometry',
            executable='amr_odometry',
            name='amr_odometry',
            output='screen',
            parameters=[param_file],  # Correctly specify the parameter file
        ),
    ])
