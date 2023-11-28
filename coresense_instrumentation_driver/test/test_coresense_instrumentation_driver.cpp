#include <gtest/gtest.h>
#include "coresense_instrumentation_driver/CoresenseInstrumentationDriver.hpp"

TEST(CoresenseInstrumentationDriverTest, Initialization)
{
  rclcpp::init(0, nullptr);
  auto driver =
    std::make_unique<coresense_instrumentation_driver::CoresenseInstrumentationDriver>();
  ASSERT_EQ(driver->getNumberOfNodes(), 0);
  rclcpp::shutdown();
}

TEST(CoresenseInstrumentationDriverTest, AddNodeString)
{
  rclcpp::init(0, nullptr);
  auto driver =
    std::make_unique<coresense_instrumentation_driver::CoresenseInstrumentationDriver>();
  driver->newNode("test_topic", "std_msgs/msg/String");
  ASSERT_EQ(driver->getNumberOfNodes(), 1);
  rclcpp::shutdown();
}

TEST(CoresenseInstrumentationDriverTest, AddNodeScan)
{
  rclcpp::init(0, nullptr);
  auto driver =
    std::make_unique<coresense_instrumentation_driver::CoresenseInstrumentationDriver>();
  driver->newNode("test_topic", "sensor_msgs/msg/LaserScan");
  ASSERT_EQ(driver->getNumberOfNodes(), 1);
  rclcpp::shutdown();
}

TEST(CoresenseInstrumentationDriverTest, AddNodeFail)
{
  rclcpp::init(0, nullptr);
  auto driver =
    std::make_unique<coresense_instrumentation_driver::CoresenseInstrumentationDriver>();
  driver->newNode("test_topic_2", "std_msgs/msg/Float32");
  ASSERT_EQ(driver->getNumberOfNodes(), 0);
  rclcpp::shutdown();
}

TEST(CoresenseInstrumentationDriverTest, RemoveNodeString)
{
  rclcpp::init(0, nullptr);
  auto driver =
    std::make_unique<coresense_instrumentation_driver::CoresenseInstrumentationDriver>();
  driver->newNode("test_topic", "std_msgs/msg/String");
  ASSERT_EQ(driver->getNumberOfNodes(), 1);

  driver->removeNode("test_topic");
  ASSERT_EQ(driver->getNumberOfNodes(), 0);
  rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
