import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get the package share directory
    amr_core_share_dir = get_package_share_directory('amr_core')
    amr_odometry_share_dir = get_package_share_directory('amr_odometry')
    amr_description_share_dir = get_package_share_directory('amr_description')
    amr_lidar_share_dir = get_package_share_directory('ydlidar_ros2_driver')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = 'redbot04.urdf'

    # bringup Core
    core_parameter_file = LaunchConfiguration('core_params_file')

    core_params_declare = DeclareLaunchArgument(
        'core_params_file',
        default_value=os.path.join(amr_core_share_dir, 'param', 'ammr_param.yaml'),
        description='Path to the ROS2 parameters file to use.'
    )

    # bringup Odometry
    odometry_parameter_file = LaunchConfiguration('odometry_params_file')

    odometry_params_declare = DeclareLaunchArgument(
        'odometry_params_file',
        default_value=os.path.join(amr_odometry_share_dir, 'param', 'param.yaml'),
        description='Path to the ROS2 parameters file to use.'
    )

    # bringup Description
    urdf = os.path.join(
        amr_description_share_dir,
        'urdf',
        urdf_file_name)

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    rsp_params = {'robot_description': robot_desc, 'use_sim_time': use_sim_time} 

    # bringup LiDAR

    lidar_parameter_file = LaunchConfiguration('lidar_params_file')

    lidar_params_declare = DeclareLaunchArgument('lidar_params_file',
                                           default_value=os.path.join(
                                               amr_core_share_dir, 'param', 'ydlidar.yaml'),
                                           description='FPath to the ROS2 parameters file to use.')

    lidar_driver_node = LifecycleNode(package='ydlidar_ros2_driver',
                                executable='ydlidar_ros2_driver_node',
                                name='ydlidar_ros2_driver_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[lidar_parameter_file],
                                namespace='/',
                                )

    # Path to the laser_filter.launch.py file
    laser_launch_file = os.path.join(amr_core_share_dir, 'launch', 'laser_filter.launch.py')

    # Execute
    return LaunchDescription([
        core_params_declare,
        odometry_params_declare,
        lidar_params_declare,
        Node(
            package='amr_core',
            executable='amr_core_node',
            name='amr_core',
            output='screen',
            parameters=[core_parameter_file],  # Correctly specify the parameter file
        ),
        Node(
            package='amr_odometry',
            executable='amr_odometry',
            name='amr_odometry',
            output='screen',
            parameters=[odometry_parameter_file],  # Correctly specify the parameter file
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[rsp_params]),
        lidar_driver_node,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(laser_launch_file),
        ),
    ])
