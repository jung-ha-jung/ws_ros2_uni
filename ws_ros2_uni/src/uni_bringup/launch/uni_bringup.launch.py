# Copyright 2018 Open Source Robotics Foundation, Inc.
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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    sick_scan2_dir = LaunchConfiguration(
        'sick_scan2_dir',
        default=os.path.join(get_package_share_directory('sick_scan2'), 'launch'))

#    imu_dir = LaunchConfiguration(
#        'imu_dir',
#        default=os.path.join(get_package_share_directory('bno055'), 'launch'))

    uni_interface_dir = LaunchConfiguration(
        'uni_interface_dir',
        default=os.path.join(get_package_share_directory('uni_interface'), 'launch'))

    usb_joy_dir = LaunchConfiguration(
        'usb_joy_dir',
        default=os.path.join(get_package_share_directory('usb_joy'), 'launch'))

    uni_description_dir = LaunchConfiguration(
        'uni_description_dir',
        default=os.path.join(get_package_share_directory('uni_description'), 'launch'))

    return LaunchDescription([
        # sick_scan2
         IncludeLaunchDescription(
            PythonLaunchDescriptionSource([sick_scan2_dir, '/sick_tim_5xx.launch.py']),
         ),
#        # imu
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource([imu_dir, '/bno055.launch.py']),
#        ),
        # uni_interface
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([uni_interface_dir, '/uni_interface.launch.py']),
        ),
        # usb_joy
         IncludeLaunchDescription(
            PythonLaunchDescriptionSource([usb_joy_dir, '/usb_joy.launch.py']),
         ),
        # uni_description
         IncludeLaunchDescription(
            PythonLaunchDescriptionSource([uni_description_dir, '/uni_state_publisher.launch.py']),
         ),
    ])
