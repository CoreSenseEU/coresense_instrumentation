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

#include "coresense_instrumentation_driver/InstrumentationLifecycleNode.hpp"


namespace coresense_instrumentation_driver
{

template class coresense_instrumentation_driver::InstrumentationLifecycleNode<sensor_msgs::msg::LaserScan>;
template class coresense_instrumentation_driver::InstrumentationLifecycleNode<std_msgs::msg::String>;

template<typename TopicT>
InstrumentationLifecycleNode<TopicT>::InstrumentationLifecycleNode(
  const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("lifecycle_node", "", options)
{
  declare_parameter("topic", std::string(""));
  declare_parameter("topic_type", std::string(""));

  get_parameter("topic", topic_);
  get_parameter("topic_type", topic_type_);

  RCLCPP_INFO(get_logger(), "Creating InstrumentationGeneric");
}

template<typename TopicT>
InstrumentationLifecycleNode<TopicT>::~InstrumentationLifecycleNode()
{
  RCLCPP_DEBUG(get_logger(), "Destroying InstrumentationGeneric");
}

template<typename TopicT>
std::string InstrumentationLifecycleNode<TopicT>::get_topic()
{
  return topic_;
}

template<typename TopicT>
std::string InstrumentationLifecycleNode<TopicT>::get_topic_type()
{
  return topic_type_;
}

template<typename TopicT>
typename rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
InstrumentationLifecycleNode<TopicT>::on_configure(const rclcpp_lifecycle::State &)
{
  sub_ = this->create_subscription<TopicT>(
    topic_, 10,
    [this](const typename TopicT::SharedPtr msg) {
      if (pub_) {
        pub_->publish(std::make_unique<TopicT>(*msg));
      }
    });

  pub_ = this->create_publisher<TopicT>("/coresense" + topic_, 10);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

template<typename TopicT>
typename rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
InstrumentationLifecycleNode<TopicT>::on_activate(const rclcpp_lifecycle::State &)
{
  pub_->on_activate();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

template<typename TopicT>
typename rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
InstrumentationLifecycleNode<TopicT>::on_deactivate(const rclcpp_lifecycle::State &)
{
  pub_->on_deactivate();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

template<typename TopicT>
typename rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
InstrumentationLifecycleNode<TopicT>::on_cleanup(const rclcpp_lifecycle::State &)
{
  pub_.reset();
  sub_.reset();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

template<typename TopicT>
typename rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
InstrumentationLifecycleNode<TopicT>::on_shutdown(const rclcpp_lifecycle::State &)
{
  pub_.reset();
  sub_.reset();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

} // namespace coresense_instrumentation_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(
  coresense_instrumentation_driver::InstrumentationLifecycleNode<sensor_msgs::msg::LaserScan>)
RCLCPP_COMPONENTS_REGISTER_NODE(
  coresense_instrumentation_driver::InstrumentationLifecycleNode<std_msgs::msg::String>)
