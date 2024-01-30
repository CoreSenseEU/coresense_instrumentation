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

template class coresense_instrumentation_driver::InstrumentationProducer<sensor_msgs::msg::LaserScan>;
template class coresense_instrumentation_driver::InstrumentationProducer<std_msgs::msg::String>;
template class coresense_instrumentation_driver::InstrumentationProducer<geometry_msgs::msg::Twist>;

template<typename TopicT>
InstrumentationProducer<TopicT>::InstrumentationProducer(
  const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("lifecycle_node", "", options)
{
  declare_parameter("topic", std::string(""));
  declare_parameter("topic_type", std::string(""));
  declare_parameter("type", std::string(""));

  get_parameter("topic", topic_);
  get_parameter("topic_type", topic_type_);
  get_parameter("type", type_);

  status_pub_ = this->create_publisher<coresense_instrumentation_interfaces::msg::NodeInfo>(
    "/status", 10);

  status_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    [this]() {
      publish_status();
    });

  RCLCPP_DEBUG(get_logger(), "Creating InstrumentationGeneric");
}

template<typename TopicT>
InstrumentationProducer<TopicT>::~InstrumentationProducer()
{
  RCLCPP_DEBUG(get_logger(), "Destroying InstrumentationGeneric");
}

template<typename TopicT>
std::string InstrumentationProducer<TopicT>::get_topic()
{
  return topic_;
}

template<typename TopicT>
std::string InstrumentationProducer<TopicT>::get_topic_type()
{
  return topic_type_;
}

template<typename TopicT>
void InstrumentationProducer<TopicT>::publish_status()
{
  auto status_msg = std::make_unique<coresense_instrumentation_interfaces::msg::NodeInfo>();
  auto lifecycle_state = get_current_state();

  if (std::string(this->get_namespace()) == "/") {
    status_msg->node_name = this->get_name();
  } else {
    status_msg->node_name = std::string(this->get_namespace()) + "/" + this->get_name();
  }

  status_msg->state = lifecycle_state.id();
  status_msg->stamp = this->now();

  for (const auto & entry : publishers_) {
    status_msg->topics.push_back(entry.first);
  }

  int status;
  char * demangled_name = abi::__cxa_demangle(typeid(TopicT).name(), nullptr, nullptr, &status);
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

template<typename TopicT>
typename rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
InstrumentationProducer<TopicT>::on_configure(const rclcpp_lifecycle::State &)
{
  sub_ = this->create_subscription<TopicT>(
    topic_, 10,
    [this](const typename TopicT::SharedPtr msg) {
      for (auto & pub : publishers_) {
        if (pub.second->get_subscription_count() > 0 && pub.second->is_activated()) {
          pub.second->publish(std::make_unique<TopicT>(*msg));
        }
      }
    });

  if (topic_[0] == '/') {
    topic_ = topic_.substr(1);
  }

  std::string topic;

  if (std::string(this->get_namespace()) == "/") {
    topic = "/coresense/" + topic_;
  } else {
    topic = std::string(this->get_namespace()) + "/coresense/" + topic_;
  }

  auto pub = this->create_publisher<TopicT>(topic, 10);
  publishers_.insert({topic, pub});

  std::string create_service, delete_service;

  if (std::string(this->get_namespace()) == "/") {
    create_service = std::string(this->get_name()) + "/create_publisher";
    delete_service = std::string(this->get_name()) + "/delete_publisher";
  } else {
    create_service = std::string(this->get_namespace()) + "/" + this->get_name() +
      "/create_publisher";
    delete_service = std::string(this->get_namespace()) + "/" + this->get_name() +
      "/delete_publisher";
  }

  create_publisher_service_ =
    this->create_service<coresense_instrumentation_interfaces::srv::CreatePublisher>(
    create_service,
    std::bind(
      &InstrumentationProducer<TopicT>::handleCreatePublisherRequest, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  delete_publisher_service_ =
    this->create_service<coresense_instrumentation_interfaces::srv::DeletePublisher>(
    delete_service,
    std::bind(
      &InstrumentationProducer<TopicT>::handleDeletePublisherRequest, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

template<typename TopicT>
typename rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
InstrumentationProducer<TopicT>::on_activate(const rclcpp_lifecycle::State &)
{
  for (auto & pub : publishers_) {
    pub.second->on_activate();
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

template<typename TopicT>
typename rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
InstrumentationProducer<TopicT>::on_deactivate(const rclcpp_lifecycle::State &)
{
  for (auto & pub : publishers_) {
    pub.second->on_deactivate();
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

template<typename TopicT>
typename rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
InstrumentationProducer<TopicT>::on_cleanup(const rclcpp_lifecycle::State &)
{
  for (auto & pub : publishers_) {
    pub.second.reset();
  }

  publishers_.clear();

  sub_.reset();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

template<typename TopicT>
typename rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
InstrumentationProducer<TopicT>::on_shutdown(const rclcpp_lifecycle::State &)
{
  for (auto & pub : publishers_) {
    pub.second.reset();
  }

  sub_.reset();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

template<typename TopicT>
void InstrumentationProducer<TopicT>::handleCreatePublisherRequest(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<coresense_instrumentation_interfaces::srv::CreatePublisher::Request> request,
  const std::shared_ptr<coresense_instrumentation_interfaces::srv::CreatePublisher::Response> response)
{
  (void)request_header;

  std::string new_topic = request->topic_name;

  if (new_topic[0] == '/') {
    new_topic = new_topic.substr(1);
  }

  for (auto & pub : publishers_) {
    if (pub.first == new_topic) {
      response->success = false;
      return;
    }
  }

  auto new_pub = this->create_publisher<TopicT>("/coresense/" + new_topic, 10);

  if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    new_pub->on_activate();
  }

  publishers_.insert({new_topic, new_pub});
  response->success = true;
}

template<typename TopicT>
void InstrumentationProducer<TopicT>::handleDeletePublisherRequest(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<coresense_instrumentation_interfaces::srv::DeletePublisher::Request> request,
  const std::shared_ptr<coresense_instrumentation_interfaces::srv::DeletePublisher::Response> response)
{
  (void)request_header;

  std::string remove_topic = request->topic_name;

  if (remove_topic[0] == '/') {
    remove_topic = remove_topic.substr(1);
  }

  for (auto & pub : publishers_) {
    if (pub.first == remove_topic) {
      pub.second.reset();
    }
  }

  publishers_.erase(remove_topic);
  response->success = true;
}

} // namespace coresense_instrumentation_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(
  coresense_instrumentation_driver::InstrumentationProducer<sensor_msgs::msg::LaserScan>)
RCLCPP_COMPONENTS_REGISTER_NODE(
  coresense_instrumentation_driver::InstrumentationProducer<std_msgs::msg::String>)
RCLCPP_COMPONENTS_REGISTER_NODE(
  coresense_instrumentation_driver::InstrumentationProducer<geometry_msgs::msg::Twist>)
