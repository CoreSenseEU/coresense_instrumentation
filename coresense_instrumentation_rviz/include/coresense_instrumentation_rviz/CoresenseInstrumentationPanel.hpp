// Copyright 2024 Intelligent Robotics Lab
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

#ifndef CORESENSE_INSTRUMENTATION_RVIZ__CORESENSE_INSTRUMENTATION_PANEL_HPP_
#define CORESENSE_INSTRUMENTATION_RVIZ__CORESENSE_INSTRUMENTATION_PANEL_HPP_

#include <QtWidgets>
#include <QLabel>
#include <QLineEdit>
#include <QBasicTimer>
#include <QTreeWidget>
#include <QHeaderView>
#include <QTreeWidgetItem>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTimer>
#include <QThread>
#include <QApplication>
#include <QMessageBox>
#include <QTabWidget>

#undef NO_ERROR

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rviz_common/panel.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "coresense_instrumentation_interfaces/msg/node_info.hpp"
#include "coresense_instrumentation_interfaces/srv/create_publisher.hpp"
#include "coresense_instrumentation_interfaces/srv/delete_publisher.hpp"
#include "coresense_instrumentation_interfaces/srv/create_subscriber.hpp"
#include "coresense_instrumentation_interfaces/srv/delete_subscriber.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

class QPushButton;

namespace coresense_instrumentation_rviz
{

class CoresensePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit CoresensePanel(QWidget * parent = 0);
  virtual ~CoresensePanel();

  void onInitialize() override;

protected:
  QVBoxLayout * layout_;
  QTreeWidget * tree_widget_;
  QLabel * label_info_;
  QTabWidget * tab_widget_;
  QPushButton * button_activate_;
  QPushButton * button_deactivate_;
  QPushButton * button_create_;
  QPushButton * button_delete_;
  QLineEdit * line_edit_topic_;
  QTreeWidget * topic_box_;
  QLabel * type_label_;

private slots:
  void show_info(QTreeWidgetItem * item);

private:
  void statusCallback(const coresense_instrumentation_interfaces::msg::NodeInfo::SharedPtr msg);
  void activate_node(const std::string & node_name);
  void deactivate_node(const std::string & node_name);
  void change_state(std::string node_name, std::uint8_t transition);
  void update_state(QTreeWidgetItem * item, const std::string & node_name, std::uint8_t state);
  void check_alive();
  void create_publisher(const std::string & node_name, const std::string & topic_name);
  void create_subscriber(const std::string & node_name, const std::string & topic_name);
  void delete_publisher(const std::string & node_name, const std::string & topic_name);
  void delete_subscriber(const std::string & node_name, const std::string & topic_name);
  void create_node_layout();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<coresense_instrumentation_interfaces::msg::NodeInfo>::SharedPtr status_sub_;
  rclcpp::TimerBase::SharedPtr alive_timer_;
  std::map<std::string, std::uint8_t> state_map_;
  std::map<std::string, std::vector<std::string>> topic_map_;
  std::map<std::string, builtin_interfaces::msg::Time> time_map_;
  std::vector<std::string> nodes_;
  std::thread spin_thread_;
};

}  // namespace coresense_instrumentation_rviz

#endif  //  CORESENSE_INSTRUMENTATION_RVIZ__CORESENSE_INSTRUMENTATION_PANEL_HPP_
