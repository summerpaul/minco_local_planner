/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-25 09:52:24
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-08-27 22:49:56
 */
#include "demo.h"

#include "basis/laser_scan.h"
#include "basis/time.h"
#include "module_manager/module_manager.h"
using namespace minco_local_planner::module_manager;
using namespace minco_local_planner::basis;
using namespace minco_local_planner::utils;
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

void Demo::InitRos() {
  odom_sub_ = nh_.subscribe("/odom", 1, &Demo::OdomCallback, this);
  laser_scan_sub_ = nh_.subscribe("/scan", 1, &Demo::LaserCallback, this);
  goals_pose_sub_ = nh_.subscribe("goals", 50, &Demo::GoalsCallback, this);
  goal_pose_sub_ =
      nh_.subscribe("/move_base_simple/goal", 50, &Demo::GoalCallback, this);
  cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
}

bool Demo::InitPlanner() {
  bool flag = ModuleManager::GetInstance()->Init(config_file_path_);
  return flag;
}

void Demo::OdomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  const auto x = msg->pose.pose.position.x;
  const auto y = msg->pose.pose.position.y;
  const auto v_x = msg->twist.twist.linear.x;
  const auto v_y = msg->twist.twist.linear.y;
  const auto angular_vel = msg->twist.twist.angular.z;
  const auto yaw = NormalizeAngleRad(
      2.0 * atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
  VehiclePose cur_pose;
  cur_pose.SetX(x);
  cur_pose.SetY(y);
  cur_pose.SetYaw(yaw);
  cur_pose.SetVelX(v_x);
  cur_pose.SetVelY(v_y);
  cur_pose.SetAngularVel(angular_vel);
  ModuleManager::GetInstance()->GetRuntimeManager()->UpdateVehiclePose(
      cur_pose);
}
void Demo::LaserCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
  LaserScan laser_scan;
  laser_scan.time_stemp = GetTimeNowDouble();
  laser_scan.time_increment = msg->time_increment;
  laser_scan.angle_increment = msg->angle_increment;
  laser_scan.angle_max = msg->angle_max;
  laser_scan.angle_min = msg->angle_min;
  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    laser_scan.ranges.emplace_back(msg->ranges[i]);
    laser_scan.intensities.emplace_back(msg->intensities[i]);
  }
  ModuleManager::GetInstance()->GetRuntimeManager()->UpdateScan(laser_scan);
}

void Demo::GoalsCallback(const geometry_msgs::PoseArray::ConstPtr &msg) {}
void Demo::GoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {}
