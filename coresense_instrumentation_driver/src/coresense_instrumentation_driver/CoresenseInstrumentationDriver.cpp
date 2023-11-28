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

#include "coresense_instrumentation_driver/CoresenseInstrumentationDriver.hpp"

namespace coresense_instrumentation_driver
{

CoresenseInstrumentationDriver::CoresenseInstrumentationDriver(
  const rclcpp::NodeOptions & options)
: Node("coresense_instrumentation_driver_node", options)
{
}

CoresenseInstrumentationDriver::~CoresenseInstrumentationDriver()
{
}

} // namespace coresense_instrumentation_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(coresense_instrumentation_driver::CoresenseInstrumentationDriver)
