#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include "coresense_instrumentation_driver/InstrumentationProducer.hpp"
#include "coresense_instrumentation_driver/InstrumentationConsumer.hpp"
#include "coresense_instrumentation_interfaces/srv/create_publisher.hpp"
#include "coresense_instrumentation_interfaces/srv/delete_publisher.hpp"
#include "coresense_instrumentation_interfaces/msg/node_info.hpp"

class IntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

TEST_F(IntegrationTest, LaserScanNodeLifecycle)
{
  rclcpp::executors::SingleThreadedExecutor executor;

  auto node =
    std::make_shared<coresense_instrumentation_driver::InstrumentationProducer<sensor_msgs::msg::LaserScan>>(
    rclcpp::NodeOptions().append_parameter_override(
      "topic",
      "/test_topic").append_parameter_override(
      "topic_type", "sensor_msgs::msg::LaserScan"));

  executor.add_node(node->get_node_base_interface());

  auto result = node->on_configure(rclcpp_lifecycle::State());
  ASSERT_EQ(
    result,
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  result = node->on_activate(rclcpp_lifecycle::State());
  ASSERT_EQ(
    result,
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  node->on_activate(rclcpp_lifecycle::State());
  result = node->on_deactivate(rclcpp_lifecycle::State());
  ASSERT_EQ(
    result,
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  result = node->on_cleanup(rclcpp_lifecycle::State());
  ASSERT_EQ(
    result,
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  result = node->on_shutdown(rclcpp_lifecycle::State());
  ASSERT_EQ(
    result,
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  executor.spin_once(std::chrono::seconds(1));
}

TEST_F(IntegrationTest, StringNodeLifecycle)
{
  rclcpp::executors::SingleThreadedExecutor executor;

  auto node =
    std::make_shared<coresense_instrumentation_driver::InstrumentationProducer<std_msgs::msg::String>>(
    rclcpp::NodeOptions().append_parameter_override(
      "topic",
      "/test_topic").append_parameter_override(
      "topic_type", "std_msgs::msg::String"));

  executor.add_node(node->get_node_base_interface());

  auto result = node->on_configure(rclcpp_lifecycle::State());
  ASSERT_EQ(
    result,
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  result = node->on_activate(rclcpp_lifecycle::State());
  ASSERT_EQ(
    result,
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  node->on_activate(rclcpp_lifecycle::State());
  result = node->on_deactivate(rclcpp_lifecycle::State());
  ASSERT_EQ(
    result,
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  result = node->on_cleanup(rclcpp_lifecycle::State());
  ASSERT_EQ(
    result,
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  result = node->on_shutdown(rclcpp_lifecycle::State());
  ASSERT_EQ(
    result,
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  executor.spin_once(std::chrono::seconds(1));
}

TEST_F(IntegrationTest, TwistNodeLifecycle)
{
  rclcpp::executors::SingleThreadedExecutor executor;

  auto node =
    std::make_shared<coresense_instrumentation_driver::InstrumentationConsumer<geometry_msgs::msg::Twist>>(
    rclcpp::NodeOptions().append_parameter_override(
      "topic",
      "/test_topic").append_parameter_override(
      "topic_type", "geometry_msgs::msg::Twist"));

  executor.add_node(node->get_node_base_interface());

  auto result = node->on_configure(rclcpp_lifecycle::State());
  ASSERT_EQ(
    result,
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  result = node->on_activate(rclcpp_lifecycle::State());
  ASSERT_EQ(
    result,
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  node->on_activate(rclcpp_lifecycle::State());
  result = node->on_deactivate(rclcpp_lifecycle::State());
  ASSERT_EQ(
    result,
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  result = node->on_cleanup(rclcpp_lifecycle::State());
  ASSERT_EQ(
    result,
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  result = node->on_shutdown(rclcpp_lifecycle::State());
  ASSERT_EQ(
    result,
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  executor.spin_once(std::chrono::seconds(1));
}

TEST_F(IntegrationTest, ImageNodeLifecycle)
{
  rclcpp::executors::SingleThreadedExecutor executor;

  auto node =
    std::make_shared<coresense_instrumentation_driver::InstrumentationProducer<sensor_msgs::msg::Image>>(
    rclcpp::NodeOptions().append_parameter_override(
      "topic",
      "/test_topic").append_parameter_override(
      "topic_type", "sensor_msgs::msg::Image"));

  executor.add_node(node->get_node_base_interface());

  auto result = node->on_configure(rclcpp_lifecycle::State());
  ASSERT_EQ(
    result,
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  result = node->on_activate(rclcpp_lifecycle::State());
  ASSERT_EQ(
    result,
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  node->on_activate(rclcpp_lifecycle::State());
  result = node->on_deactivate(rclcpp_lifecycle::State());
  ASSERT_EQ(
    result,
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  result = node->on_cleanup(rclcpp_lifecycle::State());
  ASSERT_EQ(
    result,
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  result = node->on_shutdown(rclcpp_lifecycle::State());
  ASSERT_EQ(
    result,
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  executor.spin_once(std::chrono::seconds(1));
}

TEST_F(IntegrationTest, CreateStringPublisher)
{
  rclcpp::executors::SingleThreadedExecutor executor;

  auto node =
    std::make_shared<coresense_instrumentation_driver::InstrumentationProducer<std_msgs::msg::String>>(
    rclcpp::NodeOptions().append_parameter_override(
      "topic",
      "/test_topic").append_parameter_override(
      "topic_type", "std_msgs::msg::String"));

  auto node_client = rclcpp::Node::make_shared("node_client");

  executor.add_node(node->get_node_base_interface());
  executor.add_node(node_client->get_node_base_interface());

  node->on_configure(rclcpp_lifecycle::State());
  node->on_activate(rclcpp_lifecycle::State());

  auto client =
    node_client->create_client<coresense_instrumentation_interfaces::srv::CreatePublisher>(
    "/lifecycle_node/create_publisher");
  client->wait_for_service();

  auto request =
    std::make_shared<coresense_instrumentation_interfaces::srv::CreatePublisher::Request>();
  auto response =
    std::make_shared<coresense_instrumentation_interfaces::srv::CreatePublisher::Response>();
  request->topic_name = "/new_test_topic";

  client->async_send_request(
    request,
    [&response](rclcpp::Client<coresense_instrumentation_interfaces::srv::CreatePublisher>::
    SharedFuture future) {
      if (future.get()) {
        response = future.get();
      }
    });

  for (int i = 0; i < 10; i++) {
    executor.spin_once(std::chrono::milliseconds(100));
  }

  ASSERT_EQ(response->success, true);
}

TEST_F(IntegrationTest, DeleteStringPublisher)
{
  rclcpp::executors::SingleThreadedExecutor executor;

  auto node =
    std::make_shared<coresense_instrumentation_driver::InstrumentationProducer<std_msgs::msg::String>>(
    rclcpp::NodeOptions().append_parameter_override(
      "topic",
      "/test_topic").append_parameter_override(
      "topic_type", "std_msgs::msg::String"));

  auto node_client = rclcpp::Node::make_shared("node_client");

  executor.add_node(node->get_node_base_interface());
  executor.add_node(node_client->get_node_base_interface());

  node->on_configure(rclcpp_lifecycle::State());
  node->on_activate(rclcpp_lifecycle::State());

  auto client_create =
    node_client->create_client<coresense_instrumentation_interfaces::srv::CreatePublisher>(
    "/lifecycle_node/create_publisher");
  client_create->wait_for_service();

  auto request_create =
    std::make_shared<coresense_instrumentation_interfaces::srv::CreatePublisher::Request>();
  auto response_create =
    std::make_shared<coresense_instrumentation_interfaces::srv::CreatePublisher::Response>();
  request_create->topic_name = "/new_test_topic";

  client_create->async_send_request(
    request_create,
    [&response_create](rclcpp::Client<coresense_instrumentation_interfaces::srv::CreatePublisher>::
    SharedFuture future) {
      if (future.get()) {
        response_create = future.get();
      }
    });

  for (int i = 0; i < 10; i++) {
    executor.spin_once(std::chrono::milliseconds(100));
  }

  ASSERT_EQ(response_create->success, true);

  auto request_delete =
    std::make_shared<coresense_instrumentation_interfaces::srv::DeletePublisher::Request>();
  auto response_delete =
    std::make_shared<coresense_instrumentation_interfaces::srv::DeletePublisher::Response>();

  auto client_delete =
    node_client->create_client<coresense_instrumentation_interfaces::srv::DeletePublisher>(
    "/lifecycle_node/delete_publisher");
  client_delete->wait_for_service();

  request_delete->topic_name = "/coresense/new_test_topic";

  client_delete->async_send_request(
    request_delete,
    [&response_delete](rclcpp::Client<coresense_instrumentation_interfaces::srv::DeletePublisher>::
    SharedFuture future) {
      if (future.get()) {
        response_delete = future.get();
      }
    });

  for (int i = 0; i < 10; i++) {
    executor.spin_once(std::chrono::milliseconds(100));
  }

  ASSERT_EQ(response_delete->success, true);
}

TEST_F(IntegrationTest, CreateTwistSubscription)
{
  rclcpp::executors::SingleThreadedExecutor executor;

  auto node =
    std::make_shared<coresense_instrumentation_driver::InstrumentationConsumer<geometry_msgs::msg::Twist>>(
    rclcpp::NodeOptions().append_parameter_override(
      "topic",
      "/test_topic").append_parameter_override(
      "topic_type", "geometry_msgs::msg::Twist"));

  auto node_client = rclcpp::Node::make_shared("node_client");

  executor.add_node(node->get_node_base_interface());
  executor.add_node(node_client->get_node_base_interface());

  node->on_configure(rclcpp_lifecycle::State());
  node->on_activate(rclcpp_lifecycle::State());

  auto client =
    node_client->create_client<coresense_instrumentation_interfaces::srv::CreateSubscriber>(
    "/lifecycle_node/create_subscriber");
  client->wait_for_service();

  auto request =
    std::make_shared<coresense_instrumentation_interfaces::srv::CreateSubscriber::Request>();
  auto response =
    std::make_shared<coresense_instrumentation_interfaces::srv::CreateSubscriber::Response>();
  request->topic_name = "/new_test_topic";

  client->async_send_request(
    request,
    [&response](rclcpp::Client<coresense_instrumentation_interfaces::srv::CreateSubscriber>::
    SharedFuture future) {
      if (future.get()) {
        response = future.get();
      }
    });

  for (int i = 0; i < 10; i++) {
    executor.spin_once(std::chrono::milliseconds(100));
  }

  ASSERT_EQ(response->success, true);
}

TEST_F(IntegrationTest, DeleteTwistSubscription)
{
  rclcpp::executors::SingleThreadedExecutor executor;

  auto node =
    std::make_shared<coresense_instrumentation_driver::InstrumentationConsumer<geometry_msgs::msg::Twist>>(
    rclcpp::NodeOptions().append_parameter_override(
      "topic",
      "/test_topic").append_parameter_override(
      "topic_type", "geometry_msgs::msg::Twist"));

  auto node_client = rclcpp::Node::make_shared("node_client");

  executor.add_node(node->get_node_base_interface());
  executor.add_node(node_client->get_node_base_interface());

  node->on_configure(rclcpp_lifecycle::State());
  node->on_activate(rclcpp_lifecycle::State());

  auto client_create =
    node_client->create_client<coresense_instrumentation_interfaces::srv::CreateSubscriber>(
    "/lifecycle_node/create_subscriber");
  client_create->wait_for_service();

  auto request_create =
    std::make_shared<coresense_instrumentation_interfaces::srv::CreateSubscriber::Request>();
  auto response_create =
    std::make_shared<coresense_instrumentation_interfaces::srv::CreateSubscriber::Response>();
  request_create->topic_name = "/new_test_topic";

  client_create->async_send_request(
    request_create,
    [&response_create](rclcpp::Client<coresense_instrumentation_interfaces::srv::CreateSubscriber>::
    SharedFuture future) {
      if (future.get()) {
        response_create = future.get();
      }
    });

  for (int i = 0; i < 10; i++) {
    executor.spin_once(std::chrono::milliseconds(100));
  }

  ASSERT_EQ(response_create->success, true);

  auto request_delete =
    std::make_shared<coresense_instrumentation_interfaces::srv::DeleteSubscriber::Request>();
  auto response_delete =
    std::make_shared<coresense_instrumentation_interfaces::srv::DeleteSubscriber::Response>();

  auto client_delete =
    node_client->create_client<coresense_instrumentation_interfaces::srv::DeleteSubscriber>(
    "/lifecycle_node/delete_subscriber");
  client_delete->wait_for_service();

  request_delete->topic_name = "/coresense/new_test_topic";

  client_delete->async_send_request(
    request_delete,
    [&response_delete](rclcpp::Client<coresense_instrumentation_interfaces::srv::DeleteSubscriber>::
    SharedFuture future) {
      if (future.get()) {
        response_delete = future.get();
      }
    });

  for (int i = 0; i < 10; i++) {
    executor.spin_once(std::chrono::milliseconds(100));
  }

  ASSERT_EQ(response_delete->success, true);
}

TEST_F(IntegrationTest, GetStatus)
{
  rclcpp::executors::SingleThreadedExecutor executor;

  auto node =
    std::make_shared<coresense_instrumentation_driver::InstrumentationProducer<std_msgs::msg::String>>(
    rclcpp::NodeOptions().append_parameter_override(
      "topic",
      "/test_topic").append_parameter_override(
      "topic_type", "std_msgs::msg::String").
    append_parameter_override("type", "Producer"));

  auto node_sub = rclcpp::Node::make_shared("node_sub");

  executor.add_node(node->get_node_base_interface());
  executor.add_node(node_sub->get_node_base_interface());

  auto node_info = std::make_shared<coresense_instrumentation_interfaces::msg::NodeInfo>();

  auto sub = node_sub->create_subscription<coresense_instrumentation_interfaces::msg::NodeInfo>(
    "/status", 10,
    [&node_info](const coresense_instrumentation_interfaces::msg::NodeInfo::SharedPtr msg) {
      node_info = std::move(msg);
    });

  for (int i = 0; i < 30; i++) {
    executor.spin_once(std::chrono::milliseconds(100));
  }

  ASSERT_EQ(node_info->node_name, "lifecycle_node");
  ASSERT_EQ(node_info->type, "Producer");
  ASSERT_EQ(node_info->state, 1);
  ASSERT_EQ(node_info->type_msg, "std_msgs::msg::String");
}

TEST_F(IntegrationTest, GetTopic)
{
  auto node =
    std::make_shared<coresense_instrumentation_driver::InstrumentationProducer<std_msgs::msg::String>>(
    rclcpp::NodeOptions().append_parameter_override(
      "topic",
      "/test_topic").append_parameter_override(
      "topic_type", "std_msgs::msg::String"));

  ASSERT_EQ(node->get_topic(), "/test_topic");
}

TEST_F(IntegrationTest, GetTopicType)
{
  auto node =
    std::make_shared<coresense_instrumentation_driver::InstrumentationProducer<std_msgs::msg::String>>(
    rclcpp::NodeOptions().append_parameter_override(
      "topic",
      "/test_topic").append_parameter_override(
      "topic_type", "std_msgs::msg::String"));

  ASSERT_EQ(node->get_topic_type(), "std_msgs::msg::String");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
