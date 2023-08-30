/**
 * @Author: Xia Yunkai
 * @Date:   2023-08-24 20:06:01
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-08-30 15:46:03
 */
#include "safety_manager.h"

#include <iostream>

#include "module_manager/module_manager.h"
#include "utils/singleton.h"
#include "utils/timer_manager.h"
using namespace std;

namespace minco_local_planner::safety_manager {

using namespace module_manager;
SafetyManager::SafetyManager() : BaseModule("SafetyManager") {}

SafetyManager::~SafetyManager() {}
bool SafetyManager::Init() {
  cfg_ = ModuleManager::GetInstance()
             ->GetConfigManager()
             ->GetSafetyManagerConfig();
  GenerateBoundingBoxes();
  vehicle_safety_status_ = SafetyStatus::STOP;
  path_safety_status_ = SafetyStatus::STOP;

  return true;
}

bool SafetyManager::Start() {
  Singleton<TimerManager>()->Schedule(
      100, std::bind(&SafetyManager::CheckBoundingBoxesTimer, this));
  return true;
}
void SafetyManager::Stop() {}

void SafetyManager::GenerateBoundingBoxes() {
  const auto y_min = -0.5 * cfg_.vehicle_width;
  const auto y_max = 0.5 * cfg_.vehicle_width;
  const auto x_min = -cfg_.back_to_center;
  const auto x_max = cfg_.vehicle_length - cfg_.back_to_center;
  vehicle_box_ = BoundingBox(x_min, x_max, y_min, y_max, "vehicle");
  vehicle_box_points_ = vehicle_box_.GetPoints();
  bouding_boxes_[BoundingBoxType::VEHICLE] = vehicle_box_;
  stop_box_ = BoundingBox(x_min, x_max, y_min, y_max, cfg_.stop_box_x_margin,
                          cfg_.stop_box_y_margin, "stop");
  bouding_boxes_[BoundingBoxType::STOP] = stop_box_;

  creep_box_ = BoundingBox(x_min, x_max, y_min, y_max, cfg_.creep_box_x_margin,
                           cfg_.creep_box_y_margin, "creep");
  bouding_boxes_[BoundingBoxType::CREEP] = creep_box_;
  slow_down_box_ =
      BoundingBox(x_min, x_max, y_min, y_max, cfg_.slow_down_box_x_margin,
                  cfg_.slow_down_box_y_margin, "slow_down");
  bouding_boxes_[BoundingBoxType::SLOW_DOWN] = slow_down_box_;
}

bool SafetyManager::CheckPose2dObs(const Pose2d& check_pt) {
  Vec2d pos = check_pt.head(2);
  double yaw = check_pt[2];
  Mat2d Rotation_matrix;
  Rotation_matrix << cos(yaw), -sin(yaw), sin(yaw), cos(yaw);

  for (int i = 0; i < 4; i++) {
    Vec2d start_pt = pos + Rotation_matrix * vehicle_box_points_[i];
    Vec2d end_pt = pos + Rotation_matrix * vehicle_box_points_[i + 1];
    RayCaster raycaster;
  }
}

void SafetyManager::UpdateCheckTraj(const Trajectory& traj) {
  std::lock_guard<std::mutex> lock(check_traj_mtx_);
  check_traj_ = traj;
  check_traj_length_ = check_traj_.Length();
}

// 检查安全轨迹
void SafetyManager::CheckTrajectoryTimer() {
  std::lock_guard<std::mutex> lock1(check_traj_mtx_);
  std::lock_guard<std::mutex> lock2(safe_check_path_mtx_);
  const int check_traj_size = check_traj_.size();
  if (check_traj_size == 0) {
    if (vehicle_safety_status_ == SafetyStatus::SAFE &&
        path_safety_status_ != SafetyStatus::SAFE) {
      ChangePathSafetyState(SafetyStatus::SAFE);
    }

    else if (vehicle_safety_status_ == SafetyStatus::STOP &&
             path_safety_status_ != SafetyStatus::STOP) {
      ChangePathSafetyState(SafetyStatus::STOP);
    }

    return;
  }

  const auto cur_pose = ModuleManager::GetInstance()
                            ->GetRuntimeManager()
                            ->GetVehiclePose()
                            .GetPos();
  auto it_nearest = check_traj_.FindNearest(
      check_traj_.begin(), check_traj_.end(), cur_pose, check_traj_length_);

  Path2d safe_check_path;
  const double it_nearest_s = it_nearest->GetS();

  for (auto it = it_nearest; it != check_traj_.end(); ++it) {
    const auto check_length = it->GetS() - it_nearest_s;
    const auto check_pose = it->GetPos();
    safe_check_path.emplace_back(check_pose);
    const auto cur_kappa = std::fabs(it->GetKappa());
    // 检查轨迹的曲率
    if (IsLarge(cur_kappa, cfg_.max_kappa)) {
      if (path_safety_status_ != SafetyStatus::OVER_MAX_KAPPA) {
        ChangePathSafetyState(SafetyStatus::OVER_MAX_KAPPA);
      }
      return;
    }
    // 安全检查范围内都内安全
    if (IsLarge(check_length, cfg_.safe_check_path_length)) {
      if (path_safety_status_ != SafetyStatus::SAFE) {
        ChangePathSafetyState(SafetyStatus::SAFE);
      }
      return;
    }
  }
}

void SafetyManager::CheckBoundingBoxesTimer() {
  const auto pointcloud =
      ModuleManager::GetInstance()->GetMapManager()->GetTransformedPointcloud();

  for (auto& pt : pointcloud.points) {
    if (stop_box_.InBoundingBox(pt.x, pt.y)) {
      ChangeVehicleSafetyState(SafetyStatus::STOP);
      return;
    }
    if (creep_box_.InBoundingBox(pt.x, pt.y)) {
      ChangeVehicleSafetyState(SafetyStatus::CREEP);
      return;
    }
    if (slow_down_box_.InBoundingBox(pt.x, pt.y)) {
      ChangeVehicleSafetyState(SafetyStatus::SLOW_DOWN);
      return;
    }
  }

  if (vehicle_safety_status_ != SafetyStatus::SAFE) {
    ChangeVehicleSafetyState(SafetyStatus::SAFE);
  }
}

void SafetyManager::ChangeVehicleSafetyState(const SafetyStatus& new_status) {
  const int pre_s = int(vehicle_safety_status_);

  int cur_s = int(new_status);

  static int dangerous_to_safe_counts = 0;
  if (cur_s > pre_s) {
    vehicle_safety_status_ = new_status;
    dangerous_to_safe_counts = 0;
    LOG_INFO("vehicle_safety_status_ form {} to {}", safety_status_str_[pre_s],
             safety_status_str_[cur_s]);
  } else if ((pre_s - cur_s) >= 1) {
    dangerous_to_safe_counts++;
    if (dangerous_to_safe_counts > cfg_.dangerous_to_safe_counts) {
      cur_s = pre_s - 1;

      vehicle_safety_status_ = SafetyStatus(cur_s);
      dangerous_to_safe_counts = 0;
      LOG_INFO("vehicle_safety_status_ form {} to {}",
               safety_status_str_[pre_s], safety_status_str_[cur_s]);
    }
  } else if (cur_s == pre_s) {
    dangerous_to_safe_counts = 0;
  }
}
void SafetyManager::ChangePathSafetyState(const SafetyStatus& new_status) {
  const int pre_s = int(path_safety_status_);
  path_safety_status_ = new_status;
  const int cur_s = int(new_status);
  LOG_INFO("path_safety_status_ form {} to {}", safety_status_str_[pre_s],
           safety_status_str_[cur_s]);
}
}  // namespace minco_local_planner::safety_manager