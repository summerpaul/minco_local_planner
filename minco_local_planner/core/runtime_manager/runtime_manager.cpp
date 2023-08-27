/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-24 17:23:51
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-08-27 22:51:30
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

  scan_active_.store(false);
  pose_active_.store(false);
  twist_active_.store(false);
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

void RuntimeManager::CheckRuntimeTimer() {
  const auto cur_t = GetTimeNowDouble();
  const auto vehicle_dt = cur_t - pose_t_;
  const auto laser_dt = cur_t - scan_t_;
  const bool miss_pose =
      (vehicle_dt > cfg_.message_wait_time && !pose_active_.load());
  const bool miss_scan =
      (laser_dt > cfg_.message_wait_time && !scan_active_.load());
  if (miss_pose && miss_scan) {
    if (status_ != RuntimeStatus::MISS_ALL) {
      ChangeStatus(RuntimeStatus::MISS_ALL);
    }
  } else if (miss_pose && !miss_scan) {
    if (status_ != RuntimeStatus::MISS_POSE) {
      ChangeStatus(RuntimeStatus::MISS_POSE);
    }
  } else if (!miss_pose && miss_scan) {
    if (status_ != RuntimeStatus::MISS_SCAN) {
      ChangeStatus(RuntimeStatus::MISS_SCAN);
    }
  } else if (!miss_pose && !miss_scan) {
    if (status_ != RuntimeStatus::NORMAL) {
      ChangeStatus(RuntimeStatus::NORMAL);
    }
  }
}

void RuntimeManager::ChangeStatus(const RuntimeStatus& new_status) {
  const static std::string status_str[4] = {"NORMAL", "MISS_POSE", "MISS_SCAN",
                                            "MISS_ALL"};

  int pre_s = int(status_);
  status_ = new_status;

  LOG_INFO("Runtime state form {} to {}", status_str[pre_s],
           status_str[int(new_status)]);
}
}  // namespace minco_local_planner::runtime_manager