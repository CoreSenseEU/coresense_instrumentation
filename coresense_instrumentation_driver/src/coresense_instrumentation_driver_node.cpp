// Copyright 2023 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rclcpp/rclcpp.hpp>
#include "coresense_instrumentation_driver/InstrumentationLifecycleNode.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  if (argc < 3) {
    RCLCPP_INFO(rclcpp::get_logger("main"), "Usage: %s <topic> <topic_type>", argv[0]);
    return 1;
  }

  std::string topic = argv[1];
  std::string topic_type = argv[2];

  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  if (topic_type == "sensor_msgs/msg/LaserScan") {
    auto node =
      std::make_shared<coresense_instrumentation_driver::InstrumentationLifecycleNode<sensor_msgs::msg::LaserScan>>();
    executor->add_node(node->get_node_base_interface());
    executor->spin();
  } else if (topic_type == "std_msgs/msg/String") {
    auto node =
      std::make_shared<coresense_instrumentation_driver::InstrumentationLifecycleNode<std_msgs::msg::String>>();
    executor->add_node(node->get_node_base_interface());
    executor->spin();
  } else if (topic_type == "sensor_msgs/msg/Image") {
    auto node =
      std::make_shared<coresense_instrumentation_driver::InstrumentationLifecycleNode<sensor_msgs::msg::Image>>();
    executor->add_node(node->get_node_base_interface());
    executor->spin();
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Unknown topic type: %s", topic_type.c_str());
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
