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

#include "coresense_instrumentation_driver/CoresenseInstrumentationDriver.hpp"

namespace coresense_instrumentation_driver
{

CoresenseInstrumentationDriver::CoresenseInstrumentationDriver(
  const rclcpp::NodeOptions & options)
: Node("coresense_instrumentation_driver_node", options)
{
  declare_parameter("topics", std::vector<std::string>());
  declare_parameter("topic_types", std::vector<std::string>());

  get_parameter("topics", topics_);
  get_parameter("topic_types", topic_types_);

  for (size_t i = 0; i < topics_.size(); i++) {

    if (topic_types_[i] == "std_msgs/msg/String") {
      auto node = std::make_shared<InstrumentationLifecycleNode<std_msgs::msg::String>>(
        "instrumentation_" + std::to_string(i), topics_[i], get_namespace());

      nodes_.push_back(node);

      executor_.add_node(node->get_node_base_interface());
    } else if (topic_types_[i] == "sensor_msgs/msg/LaserScan") {
      auto node = std::make_shared<InstrumentationLifecycleNode<sensor_msgs::msg::LaserScan>>(
        "instrumentation_" + std::to_string(i), topics_[i], get_namespace());

      nodes_.push_back(node);

      executor_.add_node(node->get_node_base_interface());
    } else {
      RCLCPP_ERROR(get_logger(), "Unknown topic type '%s'", topic_types_[i].c_str());
      exit(-1);
    }
  }

  thread_ = std::make_unique<std::thread>([&]() {executor_.spin();});
}

CoresenseInstrumentationDriver::~CoresenseInstrumentationDriver()
{
  if (thread_ && thread_->joinable()) {
    thread_->join();
  }

  RCLCPP_INFO(get_logger(), "Destroying CoresenseInstrumentationDriver");
}

} // namespace coresense_instrumentation_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(coresense_instrumentation_driver::CoresenseInstrumentationDriver)
