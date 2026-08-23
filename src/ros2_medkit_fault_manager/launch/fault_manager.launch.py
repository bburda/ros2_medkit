# Copyright 2025 mfaferek93
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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('ros2_medkit_fault_manager'),
        'config',
        'fault_manager.yaml',
    )

    configured_params = ParameterFile(
        LaunchConfiguration('params_file'),
        allow_substs=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='YAML parameter file for fault_manager_node (confirmation, healing, '
                        'snapshots, black-box rosbag). Defaults to the conservative single-node '
                        'config; the bringup passes shared params to turn on healing and capture.',
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Optional ROS 2 namespace for the fault_manager node (for example '
                        'robot1 -> /robot1/fault_manager/...). Give the gateway a matching '
                        'fault_manager.namespace parameter so it resolves the same paths.',
        ),
        Node(
            package='ros2_medkit_fault_manager',
            executable='fault_manager_node',
            name='fault_manager',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[configured_params],
        ),
    ])
