# Copyright 2018 Open Source Robotics Foundation, Inc.
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

from launch import LaunchDescription
import launch_ros.actions
import os
from launch.actions import  IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    imu = IncludeLaunchDescription(PythonLaunchDescriptionSource(
    			os.path.join(get_package_share_directory('um7_imu'),'launch/um7.launch.py')))
    
    piksi = IncludeLaunchDescription(PythonLaunchDescriptionSource(
    			os.path.join(get_package_share_directory('piksi_multi'),'launch/piksi.launch.py')))

    return LaunchDescription([
    	imu,
    	piksi           
])
