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

    names = ['scan', 'chatter', 'image']
    topics = ['/scan', '/chatter', '/image']
    types = ['sensor_msgs::msg::LaserScan', 'std_msgs::msg::String', 'sensor_msgs::msg::Image']

    composable_nodes = []
    for topic, topic_type, name in zip(topics, types, names):
        composable_node = ComposableNode(
            package='coresense_instrumentation_driver',
            plugin='coresense_instrumentation_driver::InstrumentationLifecycleNode<'
                    + topic_type + '>',
            name=name + '_node',
            namespace='coresense',
            parameters=[{'topic': topic, 'topic_type': topic_type}],
        )
        composable_nodes.append(composable_node)

    container = ComposableNodeContainer(
        name='coresense_container',
        namespace='coresense',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(container)

    return ld
