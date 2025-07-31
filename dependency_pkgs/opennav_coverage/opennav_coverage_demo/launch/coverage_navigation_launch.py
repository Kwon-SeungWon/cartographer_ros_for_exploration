#!/usr/bin/env python3

# Copyright (c) 2023 Open Navigation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    nav2_bringup_dir = get_package_share_directory('amr_navigation')
    opennav_coverage_demo_dir = get_package_share_directory('opennav_coverage_demo')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_file = LaunchConfiguration('map', default='')
    params_file = LaunchConfiguration('params_file')
    coverage_params_file = LaunchConfiguration('coverage_params_file')
    
    # Get default map file path
    default_map_file = '/home/caselab/catkin_ws/src/Wheelchair_Robot_ROS2_Project/data/map_data/maps/geumjeong_show.yaml'
    
    # Default parameters
    autostart = True
    use_sim_time_param = False
    
    # Create parameter substitutions
    param_substitutions = {
        'use_sim_time': str(use_sim_time_param),
        'autostart': str(autostart),
        'yaml_filename': map_file
    }

    # Configure parameters
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)
    
    configured_coverage_params = ParameterFile(
        RewrittenYaml(
            source_file=coverage_params_file,
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    # Environment variable
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=default_map_file,
        description='Full path to map file to load')
    
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(opennav_coverage_demo_dir, 'demo_params.yaml'),
        description='Full path to the ROS2 parameters file to use')
    
    declare_coverage_params_file = DeclareLaunchArgument(
        'coverage_params_file',
        default_value=os.path.join(opennav_coverage_demo_dir, 'demo_params.yaml'),
        description='Full path to the coverage parameters file to use')

    # Remappings
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Create Nav2 container
    create_container = Node(
        name='nav2_container',
        package='rclcpp_components',
        executable='component_container_isolated',
        parameters=[configured_params, {'autostart': autostart}],
        remappings=remappings,
        output='screen')

    # Load basic navigation nodes
    load_basic_nav_nodes = LoadComposableNodes(
        target_container='nav2_container',
        composable_node_descriptions=[
            # Map Server
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                name='map_server',
                parameters=[configured_params, {'yaml_filename': map_file}],
                remappings=remappings),
            
            # AMCL
            ComposableNode(
                package='nav2_amcl',
                plugin='nav2_amcl::AmclNode',
                name='amcl',
                parameters=[configured_params],
                remappings=remappings),
            
            # Lifecycle Manager for Localization
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_localization',
                parameters=[{'use_sim_time': use_sim_time_param,
                           'autostart': autostart,
                           'node_names': ['map_server', 'amcl']}]),
        ],
    )

    # Load coverage navigation nodes
    load_coverage_nodes = LoadComposableNodes(
        target_container='nav2_container',
        composable_node_descriptions=[
            # Controller Server
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[configured_coverage_params],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            
            # Coverage Server
            ComposableNode(
                package='opennav_coverage',
                plugin='opennav_coverage::CoverageServer',
                name='coverage_server',
                parameters=[configured_coverage_params],
                remappings=remappings),
            
            # BT Navigator
            ComposableNode(
                package='backported_bt_navigator',
                plugin='backported_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[configured_coverage_params],
                remappings=remappings),
            
            # Velocity Smoother
            ComposableNode(
                package='nav2_velocity_smoother',
                plugin='nav2_velocity_smoother::VelocitySmoother',
                name='velocity_smoother',
                parameters=[configured_coverage_params],
                remappings=remappings +
                           [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
            
            # Lifecycle Manager for Navigation
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=[{'use_sim_time': use_sim_time_param,
                           'autostart': autostart,
                           'node_names': ['controller_server', 'bt_navigator', 
                                         'velocity_smoother', 'coverage_server']}]),
        ],
    )

    # Create the launch description
    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_map)
    ld.add_action(declare_params_file)
    ld.add_action(declare_coverage_params_file)
    ld.add_action(create_container)
    ld.add_action(load_basic_nav_nodes)
    ld.add_action(load_coverage_nodes)
    
    return ld 