# Copyright (c) 2020 Fetullah Atas
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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch_ros.events.lifecycle import ChangeState
from launch_ros.events.lifecycle import matches_node_name
from launch_ros.event_handlers import OnStateTransition
from launch.actions import LogInfo
from launch.events import matches_action
from launch.event_handlers.on_shutdown import OnShutdown

import lifecycle_msgs.msg
import os


def generate_launch_description():
    share_dir = get_package_share_directory(
        'husky_gps_navigation')
    parameter_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    node_name = 'nav2_wp_follower_client'

    params_declare = DeclareLaunchArgument('params_file',
                                           default_value=os.path.join(
                                               share_dir, 'params', 'gps_collected_points.yaml'),
                                           description='Path to the ROS2 parameters file to use.')

    use_sim_declare = DeclareLaunchArgument('use_sim_time',
                                            default_value='false',
                                            description='Is a Gazebo simulation')

    driver_node = LifecycleNode(package='husky_gps_navigation',
                                executable='nav2_wp_follower_client',
                                name=node_name,
                                namespace='',
                                output='screen',
                                parameters=[parameter_file,
                                            {'use_sim_time', use_sim_time}],
                                )

    return LaunchDescription([
        params_declare,
        use_sim_declare,
        driver_node,
    ])
