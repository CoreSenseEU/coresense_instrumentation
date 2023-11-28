# Copyright 2023 Intelligent Robotics Lab
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
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    pkg_dir = get_package_share_directory('coresense_instrumentation_driver')
    param_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    coresense_node = ComposableNode(
        package='coresense_instrumentation_driver',
        plugin='coresense_instrumentation_driver::CoresenseInstrumentationDriver',
        name='coresense_instrumentation_driver_node',
        namespace='coresense',
        parameters=[{"topics": ["/scan", "/chatter"],
                     "topic_types": ["sensor_msgs/msg/LaserScan", "std_msgs/msg/String"]}],
    )

    container = ComposableNodeContainer(
        name='coresense_container',
        namespace='coresense',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[coresense_node],
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(container)

    return ld
