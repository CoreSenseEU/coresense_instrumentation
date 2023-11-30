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

#ifndef INSTRUMENTATION_LIFECYCLE_NODE_HPP
#define INSTRUMENTATION_LIFECYCLE_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.hpp"

namespace coresense_instrumentation_driver
{

template<typename TopicT>
class InstrumentationLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  InstrumentationLifecycleNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  virtual ~InstrumentationLifecycleNode();

  std::string get_topic();
  std::string get_topic_type();

  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturnT on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State &) override;

private:
  typename rclcpp::Subscription<TopicT>::SharedPtr sub_;
  typename rclcpp_lifecycle::LifecyclePublisher<TopicT>::SharedPtr pub_;
  std::string topic_;
  std::string topic_type_;
};

template<>
class InstrumentationLifecycleNode<sensor_msgs::msg::Image>: public rclcpp_lifecycle::LifecycleNode
{
public:
  InstrumentationLifecycleNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  virtual ~InstrumentationLifecycleNode();

  std::string get_topic();
  std::string get_topic_type();

  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturnT on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State &) override;

private:
  image_transport::Publisher pub_;
  typename rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  std::string topic_;
  std::string topic_type_;
};

} // namespace coresense_instrumentation_driver

#endif // INSTRUMENTATION_LIFECYCLE_NODE_HPP
