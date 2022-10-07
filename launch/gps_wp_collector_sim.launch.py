from launch import LaunchDescription
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('husky_gps_navigation')
    collected_points_output_path = os.path.join(
        pkg_share, 'params', 'gps_collected_points.yaml')
    output = LaunchConfiguration('output')
    declare_output_cmd = DeclareLaunchArgument(
        'output',
        default_value=collected_points_output_path)
    return LaunchDescription([
        declare_output_cmd,
        launch_ros.actions.Node(
            package='husky_gps_navigation',
            executable='gps_waypoint_collector',
            name='gps_waypoint_collector',
            output='screen',
            remappings=[('/gps', '/gps/data'),
                        ('/imu', '/imu/data')],
            # frequency as points per minute
            parameters=[{
                'frequency': 20,
                'yaml_file_out': output,
                'path_length': 50,
                'client_node_name': 'nav2_wp_follower_client'
            }],
        ),
    ])
