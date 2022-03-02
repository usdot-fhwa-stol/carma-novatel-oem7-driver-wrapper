
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


from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import SetEnvironmentVariable

import os

def generate_launch_description():
    
    # Declare arguments
    log_level = LaunchConfiguration('log_level')
    declare_log_level_arg = DeclareLaunchArgument(
        name ='log_level', default_value = 'WARN', description="Log level to print.", choices=["DEBUG","INFO","WARN","ERROR","FATAL"])

    ip_addr = LaunchConfiguration('ip_addr')
    declare_ip_addr = DeclareLaunchArgument(name = 'ip_addr', default_value ='192.168.88.29', description="The IP of the gps")

    port = LaunchConfiguration('port')
    declare_port = DeclareLaunchArgument(name = 'port', default_value='2000', description = "The port to use")

    vehicle_calibration_dir = LaunchConfiguration('vehicle_calibration_dir')
    declare_vehicle_calibration_dir_arg = DeclareLaunchArgument(
        name = 'vehicle_calibration_dir', 
        default_value = "/opt/carma/vehicle/calibration",
        description = "Path to folder containing vehicle calibration directories"
    )

    novatel_params_override_env = SetEnvironmentVariable('NOVATEL_OEM7_DRIVER_PARAM_OVERRIDES_PATH', [vehicle_calibration_dir, "novatel_oem7_driver", "parameter_overrides.yaml"])

    # Define novatel driver node
    novatel_driver_pkg = get_package_share_directory('novatel_oem7_driver')
    novatel_driver_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(['/', novatel_driver_pkg, '/launch','/oem7_net.launch.py']),
                launch_arguments={'oem7_ip_addr': ip_addr, 'oem7_port' : port, 'oem7_if': 'Oem7ReceiverTcp'}.items(),
    )
    

    # Add novatel wrapper to carma container
    param_file_path = os.path.join(
        get_package_share_directory('carma_novatel_driver_wrapper'),'config/parameters.yaml')
    
    novatel_wrapper_container = ComposableNodeContainer(
        package = 'carma_ros2_utils',
        name ='carma_novatel_driver_wrapper_container',
        namespace = '/',
        executable = 'carma_component_container_mt',
        composable_node_descriptions=[

            # Launch the core nodes
            ComposableNode(
                package = 'carma_novatel_driver_wrapper',
                plugin='carma_novatel_driver_wrapper::CarmaNovatelDriverWrapper',
                name='carma_novatel_driver_wrapper_node',
                extra_arguments=[
                    {'use_intra_process_comms': True},
                    {'--log-level' : log_level}
                ],
                parameters=[ param_file_path ]
            )
        ]
    )

    return LaunchDescription(
        [
            declare_log_level_arg,
            declare_vehicle_calibration_dir_arg,
            novatel_params_override_env,
            declare_ip_addr,
            declare_port, 
            novatel_wrapper_container,
            novatel_driver_node
        ]
    )
