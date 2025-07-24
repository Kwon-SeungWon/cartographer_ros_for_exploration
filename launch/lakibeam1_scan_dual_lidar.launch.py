import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # LiDAR0 파라미터 선언
    declare_frame_id0 = DeclareLaunchArgument('frame_id0', default_value='lidar_l_link', description='LiDAR0 TF frame')
    declare_output0 = DeclareLaunchArgument('output_topic0', default_value='scan0', description='LiDAR0 topic')
    declare_inverted0 = DeclareLaunchArgument('inverted0', default_value='false', description='LiDAR0 invert flag')
    declare_hostip0 = DeclareLaunchArgument('hostip0', default_value='192.168.0.81', description='LiDAR0 host IP')
    declare_sensorip0 = DeclareLaunchArgument('sensorip0', default_value='192.168.0.85', description='LiDAR0 sensor IP')
    declare_port0 = DeclareLaunchArgument('port0', default_value='"2368"', description='LiDAR0 port')
    declare_angle_off0 = DeclareLaunchArgument('angle_offset0', default_value='5', description='LiDAR0 angle offset')
    declare_scanfreq0 = DeclareLaunchArgument('scanfreq0', default_value='"20"', description='LiDAR0 scan frequency')
    declare_filter0 = DeclareLaunchArgument('filter0', default_value='"3"', description='LiDAR0 filter option')
    declare_enable0 = DeclareLaunchArgument('laser_enable0', default_value='"true"', description='LiDAR0 enable')
    declare_start0 = DeclareLaunchArgument('scan_range_start0', default_value='"45"', description='LiDAR0 scan start')
    declare_stop0 = DeclareLaunchArgument('scan_range_stop0', default_value='"315"', description='LiDAR0 scan stop')

    # LiDAR1 파라미터 선언
    declare_frame_id1 = DeclareLaunchArgument('frame_id1', default_value='lidar_r_link', description='LiDAR1 TF frame')
    declare_output1 = DeclareLaunchArgument('output_topic1', default_value='scan1', description='LiDAR1 topic')
    declare_inverted1 = DeclareLaunchArgument('inverted1', default_value='false', description='LiDAR1 invert flag')
    declare_hostip1 = DeclareLaunchArgument('hostip1', default_value='192.168.0.81', description='LiDAR1 host IP')
    declare_sensorip1 = DeclareLaunchArgument('sensorip1', default_value='192.168.0.86', description='LiDAR1 sensor IP')
    declare_port1 = DeclareLaunchArgument('port1', default_value='"2369"', description='LiDAR1 port')
    declare_angle_off1 = DeclareLaunchArgument('angle_offset1', default_value='0', description='LiDAR1 angle offset')
    declare_scanfreq1 = DeclareLaunchArgument('scanfreq1', default_value='"20"', description='LiDAR1 scan frequency')
    declare_filter1 = DeclareLaunchArgument('filter1', default_value='"3"', description='LiDAR1 filter option')
    declare_enable1 = DeclareLaunchArgument('laser_enable1', default_value='"true"', description='LiDAR1 enable')
    declare_start1 = DeclareLaunchArgument('scan_range_start1', default_value='"45"', description='LiDAR1 scan start')
    declare_stop1 = DeclareLaunchArgument('scan_range_stop1', default_value='"315"', description='LiDAR1 scan stop')

    # LaunchConfigurations
    # LaunchConfiguration 바인딩
    frame_id0 = LaunchConfiguration('frame_id0')
    output_topic0 = LaunchConfiguration('output_topic0')
    inverted0 = LaunchConfiguration('inverted0')
    hostip0 = LaunchConfiguration('hostip0')
    sensorip0 = LaunchConfiguration('sensorip0')
    port0 = LaunchConfiguration('port0')
    angle_offset0 = LaunchConfiguration('angle_offset0')
    scanfreq0 = LaunchConfiguration('scanfreq0')
    filter0 = LaunchConfiguration('filter0')
    laser_enable0 = LaunchConfiguration('laser_enable0')
    scan_range_start0 = LaunchConfiguration('scan_range_start0')
    scan_range_stop0 = LaunchConfiguration('scan_range_stop0')

    frame_id1 = LaunchConfiguration('frame_id1')
    output_topic1 = LaunchConfiguration('output_topic1')
    inverted1 = LaunchConfiguration('inverted1')
    hostip1 = LaunchConfiguration('hostip1')
    sensorip1 = LaunchConfiguration('sensorip1')
    port1 = LaunchConfiguration('port1')
    angle_offset1 = LaunchConfiguration('angle_offset1')
    scanfreq1 = LaunchConfiguration('scanfreq1')
    filter1 = LaunchConfiguration('filter1')
    laser_enable1 = LaunchConfiguration('laser_enable1')
    scan_range_start1 = LaunchConfiguration('scan_range_start1')
    scan_range_stop1 = LaunchConfiguration('scan_range_stop1')

    # LiDAR0 노드 정의
    node0 = Node(
        package='lakibeam1', executable='lakibeam1_scan_node', name='richbeam_lidar_node0', output='screen',
        parameters=[{
            'frame_id': frame_id0,
            'output_topic': output_topic0,
            'inverted': inverted0,
            'hostip': hostip0,
            'sensorip': sensorip0,
            'port': port0,
            'angle_offset': angle_offset0,
            'scanfreq': scanfreq0,
            'filter': filter0,
            'laser_enable': laser_enable0,
            'scan_range_start': scan_range_start0,
            'scan_range_stop': scan_range_stop0,
        }],
        remappings=[('/richbeam_lidar/scan0', '/scan0')]
    )

    # LiDAR1 노드 정의
    node1 = Node(
        package='lakibeam1', executable='lakibeam1_scan_node', name='richbeam_lidar_node1', output='screen',
        parameters=[{
            'frame_id': frame_id1,
            'output_topic': output_topic1,
            'inverted': inverted1,
            'hostip': hostip1,
            'sensorip': sensorip1,
            'port': port1,
            'angle_offset': angle_offset1,
            'scanfreq': scanfreq1,
            'filter': filter1,
            'laser_enable': laser_enable1,
            'scan_range_start': scan_range_start1,
            'scan_range_stop': scan_range_stop1,
        }],
        remappings=[('/richbeam_lidar/scan1', '/scan1')]
    )
    lakibeam1_pcd_dir = get_package_share_directory('lakibeam1')
    rviz_config_dir = os.path.join(lakibeam1_pcd_dir,'rviz','lakibeam1_scan_dual.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen')




        # LaunchDescription 조립
    ld = LaunchDescription()
    # LiDAR0 선언
    ld.add_action(declare_frame_id0)
    ld.add_action(declare_output0)
    ld.add_action(declare_inverted0)
    ld.add_action(declare_hostip0)
    ld.add_action(declare_sensorip0)
    ld.add_action(declare_port0)
    ld.add_action(declare_angle_off0)
    ld.add_action(declare_scanfreq0)
    ld.add_action(declare_filter0)
    ld.add_action(declare_enable0)
    ld.add_action(declare_start0)
    ld.add_action(declare_stop0)
    # LiDAR1 선언
    ld.add_action(declare_frame_id1)
    ld.add_action(declare_output1)
    ld.add_action(declare_inverted1)
    ld.add_action(declare_hostip1)
    ld.add_action(declare_sensorip1)
    ld.add_action(declare_port1)
    ld.add_action(declare_angle_off1)
    ld.add_action(declare_scanfreq1)
    ld.add_action(declare_filter1)
    ld.add_action(declare_enable1)
    ld.add_action(declare_start1)
    ld.add_action(declare_stop1)
    # ld.add_action(rviz_node)
    ld.add_action(node0)
    ld.add_action(node1)

    return ld
