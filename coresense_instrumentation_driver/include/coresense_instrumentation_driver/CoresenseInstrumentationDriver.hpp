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

#ifndef CORESENSE_INSTRUMENTATION_DRIVER_HPP
#define CORESENSE_INSTRUMENTATION_DRIVER_HPP

#include "rclcpp/rclcpp.hpp"
#include "coresense_instrumentation_driver/instrumentation_utils/InstrumentationLifecycleNode.hpp"


namespace coresense_instrumentation_driver
{

class CoresenseInstrumentationDriver : public rclcpp::Node
{
public:
  explicit CoresenseInstrumentationDriver(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~CoresenseInstrumentationDriver();

  int getNumberOfNodes();
  void newNode(const std::string & topic, const std::string & topic_type);
  void removeNode(const std::string & topic);

private:
  void initializeNodes();
  void addNode(const std::string & topic, const std::string & topic_type);

  rclcpp::executors::MultiThreadedExecutor executor_;
  std::vector<std::shared_ptr<ICell>> nodes_;
  std::vector<std::string> topics_;
  std::vector<std::string> topic_types_;
  std::unique_ptr<std::thread> thread_;
};

} // namespace coresense_instrumentation_driver

#endif // CORESENSE_INSTRUMENTATION_DRIVER_HPP
