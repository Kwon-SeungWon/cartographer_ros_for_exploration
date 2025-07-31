import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction  
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def check_param_file(context, *args, **kwargs):
    param_path = LaunchConfiguration('params_file').perform(context)
    if not os.path.isfile(param_path):
        raise FileNotFoundError(f"[Launch] Parameter file not found: {param_path}")
    return []

def generate_launch_description():
    # Get the package share directory
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value='src/AMMR_project/parameter_file/ammr.yaml',
        description='Relative path to the unified parameter YAML file'
    )

    param_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        declare_params,
        OpaqueFunction(function=check_param_file),  # (optional) path check
        # Redbot02 core node
        Node(
            package='amr_core',
            executable='amr_core_node',
            name='amr_core',
            output='screen',
            parameters=[param_file],  # Correctly specify the parameter file
        ),
    ])
