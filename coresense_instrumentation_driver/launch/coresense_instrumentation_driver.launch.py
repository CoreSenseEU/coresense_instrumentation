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

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    names = ['scan_raw', 'nav_vel', 'image_raw']
    topics = ['/scan_raw', '/nav_vel', '/head_front_camera/rgb/image_raw']
    msgs = ['sensor_msgs::msg::LaserScan', 'geometry_msgs::msg::Twist', 'sensor_msgs::msg::Image']
    node_types = ['Producer', 'Consumer', 'Producer']
    ns = ''

    composable_nodes = []
    for topic, msg, name, type in zip(topics, msgs, names, node_types):
        composable_node = ComposableNode(
            package='coresense_instrumentation_driver',
            plugin='coresense_instrumentation_driver::Instrumentation' + type + '<'
                    + msg + '>',
            name=name + '_node',
            namespace=ns,
            parameters=[{'topic': topic, 'topic_type': msg, 'type': type}],
        )
        composable_nodes.append(composable_node)

    container = ComposableNodeContainer(
        name='coresense_container',
        namespace=ns,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(container)

    return ld
