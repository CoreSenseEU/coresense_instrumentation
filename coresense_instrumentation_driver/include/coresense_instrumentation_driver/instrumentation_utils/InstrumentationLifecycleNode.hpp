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
#include "coresense_instrumentation_driver/instrumentation_utils/ICell.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace coresense_instrumentation_driver
{

template<typename TopicT>
class InstrumentationLifecycleNode : public rclcpp_lifecycle::LifecycleNode, public ICell
{
public:
  InstrumentationLifecycleNode(
    const std::string & node_name,
    const std::string & topic,
    const std::string & ns = "",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : rclcpp_lifecycle::LifecycleNode(node_name, ns, options)
    {
      sub_ = this->create_subscription<TopicT>(
        topic, 10,
        [this](const typename TopicT::SharedPtr msg) {
          if (pub_) {
              
            pub_->publish(std::make_unique<TopicT>(*msg));
          }
        });

      pub_ = this->create_publisher<TopicT>(ns + topic, 10);

      RCLCPP_INFO(get_logger(), "Creating InstrumentationLifecycleNode");
    }

  virtual ~InstrumentationLifecycleNode() {
    RCLCPP_INFO(get_logger(), "Destroying InstrumentationLifecycleNode");
  }

private:
  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT
  on_configure(const rclcpp_lifecycle::State &) override
  {
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT
  on_activate(const rclcpp_lifecycle::State &) override
  {
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT
  on_deactivate(const rclcpp_lifecycle::State &) override
  {
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT
  on_cleanup(const rclcpp_lifecycle::State &) override
  {
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT
  on_shutdown(const rclcpp_lifecycle::State &) override
  {
    return CallbackReturnT::SUCCESS;
  }

  void process() override { }

  typename rclcpp::Subscription<TopicT>::SharedPtr sub_;
  typename rclcpp::Publisher<TopicT>::SharedPtr pub_;
};

} // namespace coresense_instrumentation_driver

#endif // INSTRUMENTATION_LIFECYCLE_NODE_HPP