import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_path = LaunchConfiguration('map_path')
    fast_lio_config_file = LaunchConfiguration('fast_lio_config_file')
    relocalization_config_file = LaunchConfiguration('relocalization_config_file')
    rviz_cfg = LaunchConfiguration('rviz_cfg')
    require_initial_pose = LaunchConfiguration('require_initial_pose')

    pb_rm_share = get_package_share_directory('pb_rm_simulation')
    fast_lio_share = get_package_share_directory('fast_lio')
    relocalization_share = get_package_share_directory('scan_to_map_relocalization')

    default_map_path = os.path.abspath(
        os.path.join(os.getcwd(), 'maps', 'localization_map.pcd'))
    default_rviz_cfg = os.path.join(fast_lio_share, 'rviz', 'fastlio.rviz')
    default_relocalization_config_file = os.path.join(
        relocalization_share, 'config', 'relocalization.yaml')

    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pb_rm_share, 'launch', 'rm_simulation.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': 'RMUC2026',
            'rviz': 'false',
        }.items(),
    )

    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_share, 'launch', 'mapping.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'config_file': fast_lio_config_file,
            'rviz': 'false',
        }.items(),
    )

    relocalization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(relocalization_share, 'launch', 'relocalization.launch.py')),
        launch_arguments={
            'config_file': relocalization_config_file,
            'use_sim_time': use_sim_time,
            'map_path': map_path,
            'odom_topic': '/Odometry',
            'scan_topic': '/cloud_registered_body',
            'require_initial_pose': require_initial_pose,
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock for every launched node'),
        DeclareLaunchArgument(
            'map_path',
            default_value=default_map_path,
            description='PCD map used by scan-to-map relocalization'),
        DeclareLaunchArgument(
            'fast_lio_config_file',
            default_value='mid360.yaml',
            description='FAST_LIO config file in the fast_lio config directory'),
        DeclareLaunchArgument(
            'relocalization_config_file',
            default_value=default_relocalization_config_file,
            description='YAML parameter file for scan-to-map relocalization'),
        DeclareLaunchArgument(
            'require_initial_pose',
            default_value='true',
            description='Wait for RViz /initialpose before relocalization starts'),
        DeclareLaunchArgument(
            'rviz_cfg',
            default_value=default_rviz_cfg,
            description='RViz config file'),
        simulation_launch,
        fast_lio_launch,
        relocalization_launch,
        rviz_node,
    ])
