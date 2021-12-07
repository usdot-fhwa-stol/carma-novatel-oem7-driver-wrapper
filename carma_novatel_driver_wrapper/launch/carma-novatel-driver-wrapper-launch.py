
#  Copyright (C) 2021 LEIDOS.

#  Licensed under the Apache License, Version 2.0 (the "License"); you may not
#  use this file except in compliance with the License. You may obtain a copy of
#  the License at

#  http://www.apache.org/licenses/LICENSE-2.0

#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
#  License for the specific language governing permissions and limitations under
#  the License.

import launch
import launch.actions
from launch.actions.declare_launch_argument import DeclareLaunchArgument
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

#Nodes
carma_novatel_driver_wrapper = Node(
    package = 'carma_novatel_driver_wrapper',
    executable = 'carma_novatel_wrapper_node',
    name='carma_novatel_driver_wrapper'
)

novatel_driver_pkg = get_package_share_directory('novatel_oem7_driver')
novatel_oem7_driver = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(['/',novatel_driver_pkg, '/launch','/oem7_net.launch.py']),
    launch_arguments={'oem7_ip_addr': '192.168.74.10', 'oem7_port' : '2000', 'oem7_if': 'Oem7ReceiverTcp'}.items(),
)


def generate_launch_description():
    return LaunchDescription(
        [
            carma_novatel_driver_wrapper,
            novatel_oem7_driver
        ]
    )
