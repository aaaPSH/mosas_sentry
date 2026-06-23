import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('scan_to_map_relocalization')
    default_map_path = os.path.abspath(
        os.path.join(os.getcwd(), 'maps', 'localization_map.pcd'))
    default_config_file = os.path.join(
        package_share, 'config', 'relocalization.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config_file,
            description='YAML parameter file for scan-to-map relocalization'),
        DeclareLaunchArgument(
            'map_path',
            default_value=default_map_path,
            description='PCD map used as the scan-to-map relocalization target'),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/Odometry',
            description='FAST_LIO odometry topic. Use /odom if your system remaps it.'),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/cloud_registered_body',
            description='Undistorted scan in body/LiDAR frame from FAST_LIO'),
        DeclareLaunchArgument(
            'output_odom_topic',
            default_value='/relocalization/odom',
            description='Corrected map-frame odometry topic'),
        DeclareLaunchArgument(
            'require_initial_pose',
            default_value='true',
            description='Wait for RViz /initialpose before relocalization starts'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'),
        Node(
            package='scan_to_map_relocalization',
            executable='scan_to_map_relocalization',
            name='scan_to_map_relocalization',
            output='screen',
            parameters=[LaunchConfiguration('config_file'), {
                'map_path': LaunchConfiguration('map_path'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'scan_topic': LaunchConfiguration('scan_topic'),
                'output_odom_topic': LaunchConfiguration('output_odom_topic'),
                'require_initial_pose': LaunchConfiguration('require_initial_pose'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])
