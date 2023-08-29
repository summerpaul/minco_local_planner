/**
 * @Author: Xia Yunkai
 * @Date:   2023-08-24 20:06:01
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-08-29 20:10:30
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
      100, std::bind(&SafetyManager::CheckBoundingBoxes, this));
  return true;
}
void SafetyManager::Stop() {}

void SafetyManager::GenerateBoundingBoxes() {
  const auto y_min = -0.5 * cfg_.vehicle_width;
  const auto y_max = 0.5 * cfg_.vehicle_width;
  const auto x_min = -cfg_.back_to_center;
  const auto x_max = cfg_.vehicle_length - cfg_.back_to_center;
  // 车辆外轮廓
  //   vehicle_box_ = ;
  bouding_boxes_[BoundingBoxType::VEHICLE] =
      BoundingBox(x_min, x_max, y_min, y_max, "vehicle");

  bouding_boxes_[BoundingBoxType::STOP] =
      BoundingBox(x_min, x_max, y_min, y_max, cfg_.stop_box_x_margin,
                  cfg_.stop_box_y_margin, "stop");
  bouding_boxes_[BoundingBoxType::CREEP] =
      BoundingBox(x_min, x_max, y_min, y_max, cfg_.creep_box_x_margin,
                  cfg_.creep_box_y_margin, "creep");

  bouding_boxes_[BoundingBoxType::SLOW_DOWN] =
      BoundingBox(x_min, x_max, y_min, y_max, cfg_.slow_down_box_x_margin,
                  cfg_.slow_down_box_y_margin, "slow_down");
}

void SafetyManager::CheckBoundingBoxes() {
  const auto pointcloud =
      ModuleManager::GetInstance()->GetMapManager()->GetTransformedPointcloud();

  auto stop_box = bouding_boxes_[BoundingBoxType::STOP];
  auto creep_box = bouding_boxes_[BoundingBoxType::CREEP];
  auto slow_down_box = bouding_boxes_[BoundingBoxType::SLOW_DOWN];

  for (auto& pt : pointcloud.points) {
    if (stop_box.InBoundingBox(pt.x, pt.y)) {
      ChangeVehicleSafetyState(SafetyStatus::STOP);
      return;
    }
    if (creep_box.InBoundingBox(pt.x, pt.y)) {
      ChangeVehicleSafetyState(SafetyStatus::CREEP);
      return;
    }
    if (slow_down_box.InBoundingBox(pt.x, pt.y)) {
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