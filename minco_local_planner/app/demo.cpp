/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-25 09:52:24
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-08-27 21:30:08
 */
#include "demo.h"

#include "module_manager/module_manager.h"
using namespace minco_local_planner::module_manager;
Demo::Demo(const std::string config_file_path)
    : config_file_path_(config_file_path) {}

bool Demo::Init() {
  if (!InitPlanner()) {
    return false;
  }
  InitRos();
  return true;
}
void Demo::Run() {
  ModuleManager::GetInstance()->Run();
  ros::spin();
}

void Demo::InitRos() {}

bool Demo::InitPlanner() {
  bool flag = ModuleManager::GetInstance()->Init(config_file_path_);
  return flag;
}

void Demo::OdomCallback(const nav_msgs::Odometry::ConstPtr &msg) {}
void Demo::LaserCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {}

void Demo::GoalsCallback(const geometry_msgs::PoseArray::ConstPtr &msg) {}
void Demo::GoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {}
