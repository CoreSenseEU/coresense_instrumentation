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

#ifndef INSTRUMENTATION_PRODUCER_HPP
#define INSTRUMENTATION_PRODUCER_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "coresense_instrumentation_interfaces/srv/create_publisher.hpp"
#include "coresense_instrumentation_interfaces/srv/delete_publisher.hpp"
#include "coresense_instrumentation_interfaces/msg/node_info.hpp"
#include <cxxabi.h>

namespace coresense_instrumentation_driver
{

using std::placeholders::_1;

template<typename TopicT>
class InstrumentationProducer : public rclcpp_lifecycle::LifecycleNode
{
public:
  InstrumentationProducer(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  virtual ~InstrumentationProducer();

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
  rclcpp::Publisher<coresense_instrumentation_interfaces::msg::NodeInfo>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  void handleCreatePublisherRequest(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<coresense_instrumentation_interfaces::srv::CreatePublisher::Request> request,
    const std::shared_ptr<coresense_instrumentation_interfaces::srv::CreatePublisher::Response> response);

  void handleDeletePublisherRequest(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<coresense_instrumentation_interfaces::srv::DeletePublisher::Request> request,
    const std::shared_ptr<coresense_instrumentation_interfaces::srv::DeletePublisher::Response> response);

  void publish_status();

  rclcpp::Service<coresense_instrumentation_interfaces::srv::CreatePublisher>::SharedPtr
    create_publisher_service_;
  rclcpp::Service<coresense_instrumentation_interfaces::srv::DeletePublisher>::SharedPtr
    delete_publisher_service_;

  std::unordered_map<std::string,
    typename rclcpp_lifecycle::LifecyclePublisher<TopicT>::SharedPtr> publishers_;

  std::string topic_;
  std::string topic_type_;
};

template<>
class InstrumentationProducer<sensor_msgs::msg::Image>: public rclcpp_lifecycle::LifecycleNode
{
public:
  InstrumentationProducer(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  virtual ~InstrumentationProducer();

  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturnT on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State &) override;

  std::string get_topic();
  std::string get_topic_type();

private:
  rclcpp::Publisher<coresense_instrumentation_interfaces::msg::NodeInfo>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

  void handleCreatePublisherRequest(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<coresense_instrumentation_interfaces::srv::CreatePublisher::Request> request,
    const std::shared_ptr<coresense_instrumentation_interfaces::srv::CreatePublisher::Response> response);

  void handleDeletePublisherRequest(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<coresense_instrumentation_interfaces::srv::DeletePublisher::Request> request,
    const std::shared_ptr<coresense_instrumentation_interfaces::srv::DeletePublisher::Response> response);

  void publish_status();

  rclcpp::Service<coresense_instrumentation_interfaces::srv::CreatePublisher>::SharedPtr
    create_publisher_service_;
  rclcpp::Service<coresense_instrumentation_interfaces::srv::DeletePublisher>::SharedPtr
    delete_publisher_service_;

  std::unordered_map<std::string, image_transport::Publisher> publishers_;

  rclcpp::Node::SharedPtr node_;
  image_transport::Subscriber sub_;
  std::string topic_;
  std::string topic_type_;
};

} // namespace coresense_instrumentation_driver

#endif // INSTRUMENTATION_PRODUCER_HPP
