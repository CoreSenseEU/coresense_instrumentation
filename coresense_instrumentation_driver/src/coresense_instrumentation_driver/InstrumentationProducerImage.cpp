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

#include "coresense_instrumentation_driver/InstrumentationProducer.hpp"

namespace coresense_instrumentation_driver
{

template class coresense_instrumentation_driver::InstrumentationProducer<sensor_msgs::msg::Image>;

InstrumentationProducer<sensor_msgs::msg::Image>::InstrumentationProducer(
  const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("image_node", "", options)
{
  declare_parameter("topic", std::string(""));
  declare_parameter("topic_type", std::string(""));
  declare_parameter("type", std::string(""));

  get_parameter("topic", topic_);
  get_parameter("topic_type", topic_type_);
  get_parameter("type", type_);

  node_ = rclcpp::Node::make_shared("subnode");

  status_pub_ = this->create_publisher<coresense_instrumentation_interfaces::msg::NodeInfo>(
    "/status", 10);

  status_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    [this]() {
      publish_status();
    });

  RCLCPP_DEBUG(get_logger(), "Creating InstrumentationImage");
}

InstrumentationProducer<sensor_msgs::msg::Image>::~InstrumentationProducer()
{
  RCLCPP_DEBUG(get_logger(), "Destroying InstrumentationImage");
}

void InstrumentationProducer<sensor_msgs::msg::Image>::publish_status()
{
  auto status_msg = std::make_unique<coresense_instrumentation_interfaces::msg::NodeInfo>();
  auto lifecycle_state = get_current_state();
  rclcpp::spin_some(node_);

  if (std::string(this->get_namespace()) == "/") {
    status_msg->node_name = get_name();
  } else {
    status_msg->node_name = std::string(this->get_namespace()) + "/" + get_name();
  }

  status_msg->state = lifecycle_state.id();
  status_msg->stamp = this->now();

  for (const auto & entry : publishers_) {
    status_msg->topics.push_back(entry.first);
  }

  int status;
  char * demangled_name = abi::__cxa_demangle(
    typeid(sensor_msgs::msg::Image).name(), nullptr, nullptr, &status);
  std::string result(demangled_name);

  size_t pos = result.find('<');
  if (pos != std::string::npos) {
    result = result.substr(0, pos);
  }

  size_t last_underscore = result.rfind('_');
  if (last_underscore != std::string::npos) {
    result = result.substr(0, last_underscore);
  }

  std::free(demangled_name);

  status_msg->type_msg = result;
  status_msg->type = type_;

  status_pub_->publish(std::move(status_msg));
}

void InstrumentationProducer<sensor_msgs::msg::Image>::imageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  for (auto & pub : publishers_) {
    if (pub.second.getNumSubscribers() > 0) {
      pub.second.publish(msg);
    }
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
InstrumentationProducer<sensor_msgs::msg::Image>::on_configure(const rclcpp_lifecycle::State &)
{
  auto subscription_options = rclcpp::SubscriptionOptions();

  sub_ = image_transport::create_subscription(
    node_.get(),
    topic_,
    std::bind(
      &InstrumentationProducer<sensor_msgs::msg::Image>::imageCallback, this,
      std::placeholders::_1),
    "raw",
    rmw_qos_profile_sensor_data,
    subscription_options);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
InstrumentationProducer<sensor_msgs::msg::Image>::on_activate(const rclcpp_lifecycle::State &)
{
  image_transport::ImageTransport it(node_);
  std::string topic;
  std::string create_service, delete_service;

  if (topic_[0] == '/') {
    topic_ = topic_.substr(1);
  }

  if (std::string(this->get_namespace()) == "/") {
    create_service = std::string(this->get_name()) + "/create_publisher";
    delete_service = std::string(this->get_name()) + "/delete_publisher";
    topic = "/coresense/" + topic_;
  } else {
    create_service = std::string(this->get_namespace()) + "/" + this->get_name() +
      "/create_publisher";
    delete_service = std::string(this->get_namespace()) + "/" + this->get_name() +
      "/delete_publisher";
    topic = std::string(this->get_namespace()) + "/coresense/" + topic_;
  }

  auto pub = it.advertise(topic, 1);
  publishers_.insert({topic, pub});

  create_publisher_service_ =
    this->create_service<coresense_instrumentation_interfaces::srv::CreatePublisher>(
    create_service,
    std::bind(
      &InstrumentationProducer<sensor_msgs::msg::Image>::handleCreatePublisherRequest, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  delete_publisher_service_ =
    this->create_service<coresense_instrumentation_interfaces::srv::DeletePublisher>(
    delete_service,
    std::bind(
      &InstrumentationProducer<sensor_msgs::msg::Image>::handleDeletePublisherRequest, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
InstrumentationProducer<sensor_msgs::msg::Image>::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  publishers_.erase(topic_);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
InstrumentationProducer<sensor_msgs::msg::Image>::on_cleanup(const rclcpp_lifecycle::State &)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
InstrumentationProducer<sensor_msgs::msg::Image>::on_shutdown(const rclcpp_lifecycle::State &)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

std::string InstrumentationProducer<sensor_msgs::msg::Image>::get_topic()
{
  return topic_;
}

std::string InstrumentationProducer<sensor_msgs::msg::Image>::get_topic_type()
{
  return topic_type_;
}

void InstrumentationProducer<sensor_msgs::msg::Image>::handleCreatePublisherRequest(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<coresense_instrumentation_interfaces::srv::CreatePublisher::Request> request,
  const std::shared_ptr<coresense_instrumentation_interfaces::srv::CreatePublisher::Response> response)
{
  (void)request_header;

  std::string new_topic = request->topic_name;

  if (new_topic[0] == '/') {
    new_topic = new_topic.substr(1);
  }

  if (std::string(this->get_namespace()) == "/") {
    new_topic = std::string("/coresense/" + new_topic);
  } else {
    new_topic = std::string(this->get_namespace()) + "/coresense/" + new_topic;
  }

  for (auto & pub : publishers_) {
    if (pub.first == new_topic) {
      response->success = false;
      return;
    }
  }

  image_transport::ImageTransport it(node_);

  auto new_pub = it.advertise(new_topic, 1);
  publishers_.insert({new_topic, new_pub});
  response->success = true;
}

void InstrumentationProducer<sensor_msgs::msg::Image>::handleDeletePublisherRequest(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<coresense_instrumentation_interfaces::srv::DeletePublisher::Request> request,
  const std::shared_ptr<coresense_instrumentation_interfaces::srv::DeletePublisher::Response> response)
{
  (void)request_header;

  std::string remove_topic = request->topic_name;

  if (remove_topic[0] != '/') {
    remove_topic = "/" + remove_topic;
  }

  publishers_.erase(remove_topic);
  response->success = true;
}

} // namespace coresense_instrumentation_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(
  coresense_instrumentation_driver::InstrumentationProducer<sensor_msgs::msg::Image>)
