/**
 * @Author: Yunkai Xia
 * @Date:   2023-09-19 11:10:27
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-09-19 11:10:53
 */
#include "config_manager.h"
namespace minco_local_planner::config_manager {
bool ConfigManager::ParseSafetyManagerConfig(
    const Json& safety_manager_cfg_json) {
  if (!GetField(safety_manager_cfg_json, "back_to_center",
                safety_manager_cfg_->back_to_center)) {
    return false;
  }

  if (!GetField(safety_manager_cfg_json, "creep_box_x_margin",
                safety_manager_cfg_->creep_box_x_margin)) {
    return false;
  }

  if (!GetField(safety_manager_cfg_json, "creep_box_y_margin",
                safety_manager_cfg_->creep_box_y_margin)) {
    return false;
  }

  if (!GetField(safety_manager_cfg_json, "dangerous_to_safe_counts",
                safety_manager_cfg_->dangerous_to_safe_counts)) {
    return false;
  }

  if (!GetField(safety_manager_cfg_json, "dangerous_to_safe_counts",
                safety_manager_cfg_->dangerous_to_safe_counts)) {
    return false;
  }

  if (!GetField(safety_manager_cfg_json, "emergency_stop_length",
                safety_manager_cfg_->emergency_stop_length)) {
    return false;
  }

  if (!GetField(safety_manager_cfg_json, "max_kappa",
                safety_manager_cfg_->max_kappa)) {
    return false;
  }

  if (!GetField(safety_manager_cfg_json, "replan_dist",
                safety_manager_cfg_->replan_dist)) {
    return false;
  }

  if (!GetField(safety_manager_cfg_json, "safe_check_path_length",
                safety_manager_cfg_->safe_check_path_length)) {
    return false;
  }

  if (!GetField(safety_manager_cfg_json, "safe_check_sleep_time",
                safety_manager_cfg_->safe_check_sleep_time)) {
    return false;
  }

  if (!GetField(safety_manager_cfg_json, "safe_distance",
                safety_manager_cfg_->safe_distance)) {
    return false;
  }

  if (!GetField(safety_manager_cfg_json, "slow_down_box_x_margin",
                safety_manager_cfg_->slow_down_box_x_margin)) {
    return false;
  }

  if (!GetField(safety_manager_cfg_json, "slow_down_box_y_margin",
                safety_manager_cfg_->slow_down_box_y_margin)) {
    return false;
  }

  if (!GetField(safety_manager_cfg_json, "slow_down_length",
                safety_manager_cfg_->slow_down_length)) {
    return false;
  }

  if (!GetField(safety_manager_cfg_json, "stop_box_x_margin",
                safety_manager_cfg_->stop_box_x_margin)) {
    return false;
  }

  if (!GetField(safety_manager_cfg_json, "stop_box_y_margin",
                safety_manager_cfg_->stop_box_y_margin)) {
    return false;
  }

  if (!GetField(safety_manager_cfg_json, "vehicle_length",
                safety_manager_cfg_->vehicle_length)) {
    return false;
  }

  if (!GetField(safety_manager_cfg_json, "vehicle_width",
                safety_manager_cfg_->vehicle_width)) {
    return false;
  }
  return true;
}

}  // namespace minco_local_planner::config_manager