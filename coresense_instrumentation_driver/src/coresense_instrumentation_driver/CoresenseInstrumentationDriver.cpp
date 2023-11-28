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
#include <typeinfo>

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

  initializeNodes();
}

CoresenseInstrumentationDriver::~CoresenseInstrumentationDriver()
{
  executor_.cancel();
  if (thread_ && thread_->joinable()) {
    thread_->join();
  }

  RCLCPP_INFO(get_logger(), "Destroying CoresenseInstrumentationDriver");
}

void
CoresenseInstrumentationDriver::initializeNodes()
{
  for (size_t i = 0; i < topics_.size(); i++) {
    addNode(topics_[i], topic_types_[i]);
  }

  thread_ = std::make_unique<std::thread>([&]() {executor_.spin();});
}

void
CoresenseInstrumentationDriver::addNode(
  const std::string & new_topic,
  const std::string & new_topic_type)
{
  if (new_topic_type == "std_msgs/msg/String") {
    auto node = std::make_shared<InstrumentationLifecycleNode<std_msgs::msg::String>>(
      "instrumentation_dynamic", new_topic, get_namespace());

    nodes_.push_back(node);

    executor_.add_node(node->get_node_base_interface());
  } else if (new_topic_type == "sensor_msgs/msg/LaserScan") {
    auto node = std::make_shared<InstrumentationLifecycleNode<sensor_msgs::msg::LaserScan>>(
      "instrumentation_dynamic", new_topic, get_namespace());

    nodes_.push_back(node);

    executor_.add_node(node->get_node_base_interface());
  } else {
    RCLCPP_ERROR(get_logger(), "Unknown topic type '%s'", new_topic_type.c_str());
  }
}

void
CoresenseInstrumentationDriver::newNode(
  const std::string & new_topic,
  const std::string & new_topic_type)
{
  if (executor_.get_number_of_threads() > 0) {
    executor_.cancel();
    thread_->join();
  }

  addNode(new_topic, new_topic_type);

  thread_ = std::make_unique<std::thread>([&]() {executor_.spin();});
}

void CoresenseInstrumentationDriver::removeNode(const std::string & topic)
{
  auto it = std::find_if(
    nodes_.begin(), nodes_.end(),
    [&](const auto & node) {return node->get_topic() == topic;});

  if (it != nodes_.end()) {
    if (executor_.get_number_of_threads() > 0) {
      executor_.cancel();
      thread_->join();
    }

    executor_.remove_node(it->get()->get_base_interface());
    nodes_.erase(it);

    thread_ = std::make_unique<std::thread>([&]() {executor_.spin();});
  } else {
    RCLCPP_WARN(get_logger(), "Node for topic '%s' not found", topic.c_str());
  }
}

int CoresenseInstrumentationDriver::getNumberOfNodes()
{
  return nodes_.size();
}

} // namespace coresense_instrumentation_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(coresense_instrumentation_driver::CoresenseInstrumentationDriver)
