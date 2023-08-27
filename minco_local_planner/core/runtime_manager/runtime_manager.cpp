/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-24 17:23:51
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-08-27 21:21:09
 */
#include "runtime_manager.h"

#include <iostream>

#include "basis/time.h"
#include "module_manager/module_manager.h"
#include "utils/singleton.h"
#include "utils/timer_manager.h"
// #include "/timer_manager.h"
namespace minco_local_planner::runtime_manager {

RuntimeManager::RuntimeManager() : BaseModule("RunTimeManager") {}

RuntimeManager::~RuntimeManager() {}

bool RuntimeManager::Init() {
  using namespace module_manager;
  cfg_ = ModuleManager::GetInstance()
             ->GetConfigManager()
             ->GetRuntimeMangerConfig();
  status_ = RuntimeStatus::MISS_ALL;
  return true;
}

bool RuntimeManager::Start() {
  using namespace utils;
  Singleton<TimerManager>()->Schedule(
      100, std::bind(&RuntimeManager::CheckRuntimeTimer, this));

  return true;
}
void RuntimeManager::Stop() {}

void RuntimeManager::UpdatePose2d(const Pose2d& pose) {
  std::lock_guard<std::mutex> lock(pose_mtx_);
  cur_pos_.SetPose2d(pose);
  pose_t_ = GetTimeNowDouble();
}

void RuntimeManager::UpdateScan(const LaserScan& scan) {
  std::lock_guard<std::mutex> lock(scan_mtx_);
  cur_scan_ = scan;
  scan_t_ = GetTimeNowDouble();
}

void RuntimeManager::UpdateChassisTwist(const Twist2D& twist) {
  std::lock_guard<std::mutex> lock(pose_mtx_);
  cur_pos_.SetVelX(twist.v_x);
  cur_pos_.SetVelY(twist.v_y);
  cur_pos_.SetAngularVel(twist.omega);
}

void RuntimeManager::UpdateVehiclePose(const VehiclePose& pose) {
  std::lock_guard<std::mutex> lock(pose_mtx_);
  cur_pos_ = pose;
  pose_t_ = GetTimeNowDouble();
}

void RuntimeManager::UpdateTwist(const Twist2D& twist) {
  std::lock_guard<std::mutex> lock(twist_mtx_);
  cur_twist_ = twist;
  twist_t_ = GetTimeNowDouble();
}

void RuntimeManager::CheckRuntimeTimer() { LOG_INFO("in CheckRuntimeTimer"); }
}  // namespace minco_local_planner::runtime_manager