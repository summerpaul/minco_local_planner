/**
 * @Author: Yunkai Xia
 * @Date:   2023-03-08 13:28:24
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-09-02 09:30:43
 */
#include "mode_change_panel.h"

#include <QHBoxLayout>
#include <iostream>
// #include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>
namespace plugin {
ModeChangePanel::ModeChangePanel(QWidget *parent) : rviz::Panel(parent) {
  initRosConnection();

  QHBoxLayout *root_layout = new QHBoxLayout;
  manual_mode_btn_ = new QPushButton("手动模式");
  root_layout->addWidget(manual_mode_btn_);
  debug_mode_btn_ = new QPushButton("调试模式");
  root_layout->addWidget(debug_mode_btn_);
  auto_mode_btn_ = new QPushButton("自动模式");
  root_layout->addWidget(auto_mode_btn_);
  reload_config_btn_ = new QPushButton("重载配置");
  root_layout->addWidget(reload_config_btn_);

  setLayout(root_layout);

  connect(manual_mode_btn_, SIGNAL(clicked()), this, SLOT(manualModePub()));
  connect(debug_mode_btn_, SIGNAL(clicked()), this, SLOT(debugModePub()));
  connect(auto_mode_btn_, SIGNAL(clicked()), this, SLOT(autoModePub()));
  connect(reload_config_btn_, SIGNAL(clicked()), this, SLOT(reloadConfigPub()));
}

void ModeChangePanel::initRosConnection() {
  mode_change_pub_ = nh_.advertise<std_msgs::Int8>("vehicle_mode", 1);
  reload_config_pub_ = nh_.advertise<std_msgs::Empty>("reload_config", 1);
}

void ModeChangePanel::manualModePub() {
  std_msgs::Int8 mode;
  mode.data = 1;
  mode_change_pub_.publish(mode);
  ROS_INFO("set manual mode");
}
void ModeChangePanel::debugModePub() {
  std_msgs::Int8 mode;
  mode.data = 2;
  mode_change_pub_.publish(mode);
  ROS_INFO("set debug mode");
}

void ModeChangePanel::autoModePub() {
  std_msgs::Int8 mode;
  mode.data = 0;
  mode_change_pub_.publish(mode);
  ROS_INFO("set debug mode");
}

void ModeChangePanel::reloadConfigPub() {
  std_msgs::Empty msg;
  reload_config_pub_.publish(msg);
  ROS_INFO("reload_config pub ");
}
}  // namespace plugin

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(plugin::ModeChangePanel, rviz::Panel)