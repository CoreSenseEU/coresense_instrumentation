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

template class coresense_instrumentation_driver::InstrumentationLifecycleNode<sensor_msgs::msg::Image>;

InstrumentationLifecycleNode<sensor_msgs::msg::Image>::InstrumentationLifecycleNode(
  const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("image_node", "", options)
{
  declare_parameter("topic", std::string(""));
  declare_parameter("topic_type", std::string(""));

  get_parameter("topic", topic_);
  get_parameter("topic_type", topic_type_);

  RCLCPP_INFO(get_logger(), "Creating InstrumentationImage");
}


InstrumentationLifecycleNode<sensor_msgs::msg::Image>::~InstrumentationLifecycleNode()
{
  RCLCPP_DEBUG(get_logger(), "Destroying InstrumentationImage");
}

std::string InstrumentationLifecycleNode<sensor_msgs::msg::Image>::get_topic()
{
  return topic_;
}


std::string InstrumentationLifecycleNode<sensor_msgs::msg::Image>::get_topic_type()
{
  return topic_type_;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
InstrumentationLifecycleNode<sensor_msgs::msg::Image>::on_configure(const rclcpp_lifecycle::State &)
{
  sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    topic_, 10,
    [this](const sensor_msgs::msg::Image::SharedPtr msg) {
      if (pub_) {
        // pub_->publish(msg);
      }
    });

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
InstrumentationLifecycleNode<sensor_msgs::msg::Image>::on_activate(const rclcpp_lifecycle::State &)
{
  // image_transport::ImageTransport it(this->get_node_base_interface());
  // pub_ = it.advertise(topic_, 1);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
InstrumentationLifecycleNode<sensor_msgs::msg::Image>::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
InstrumentationLifecycleNode<sensor_msgs::msg::Image>::on_cleanup(const rclcpp_lifecycle::State &)
{
  sub_.reset();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
InstrumentationLifecycleNode<sensor_msgs::msg::Image>::on_shutdown(const rclcpp_lifecycle::State &)
{
  sub_.reset();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

} // namespace coresense_instrumentation_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(
  coresense_instrumentation_driver::InstrumentationLifecycleNode<sensor_msgs::msg::Image>)
