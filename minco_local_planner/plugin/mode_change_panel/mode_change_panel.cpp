/**
 * @Author: Yunkai Xia
 * @Date:   2023-03-08 13:28:24
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-03-08 13:42:10
 */
#include "mode_change_panel.h"
#include <QHBoxLayout>
#include <iostream>
// #include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
namespace plugin {
ModeChangePanel::ModeChangePanel(QWidget *parent) : rviz::Panel(parent) {
  initRosConnection();

  QHBoxLayout *root_layout = new QHBoxLayout;
  manual_mode_btn_ = new QPushButton("手动模式");
  root_layout->addWidget(manual_mode_btn_);
  debug_mode_btn_ = new QPushButton("调试模式");
  root_layout->addWidget(debug_mode_btn_);
  auto_mode_btn_= new QPushButton("自动模式");
  root_layout->addWidget(auto_mode_btn_);

  setLayout(root_layout);

  connect(manual_mode_btn_, SIGNAL(clicked()), this, SLOT(manualModePub()));
  connect(debug_mode_btn_, SIGNAL(clicked()), this, SLOT(debugModePub()));
  connect(auto_mode_btn_, SIGNAL(clicked()), this, SLOT(autoModePub()));
}

void ModeChangePanel::initRosConnection() {
  mode_change_pub_ = nh_.advertise<std_msgs::Int8>("vehicle_mode", 1);
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

void ModeChangePanel::autoModePub(){
  std_msgs::Int8 mode;
  mode.data = 0;
  mode_change_pub_.publish(mode);
  ROS_INFO("set debug mode");
}
} // namespace plugin

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(plugin::ModeChangePanel, rviz::Panel)