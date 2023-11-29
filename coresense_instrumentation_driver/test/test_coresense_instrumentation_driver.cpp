#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include "coresense_instrumentation_driver/InstrumentationLifecycleNode.hpp"

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
    std::make_shared<coresense_instrumentation_driver::InstrumentationLifecycleNode<sensor_msgs::msg::LaserScan>>(
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
    std::make_shared<coresense_instrumentation_driver::InstrumentationLifecycleNode<std_msgs::msg::String>>(
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

TEST_F(IntegrationTest, GetTopic)
{
  auto node =
    std::make_shared<coresense_instrumentation_driver::InstrumentationLifecycleNode<std_msgs::msg::String>>(
    rclcpp::NodeOptions().append_parameter_override(
      "topic",
      "/test_topic").append_parameter_override(
      "topic_type", "std_msgs::msg::String"));

  ASSERT_EQ(node->get_topic(), "/test_topic");
}

TEST_F(IntegrationTest, GetTopicType)
{
  auto node =
    std::make_shared<coresense_instrumentation_driver::InstrumentationLifecycleNode<std_msgs::msg::String>>(
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
