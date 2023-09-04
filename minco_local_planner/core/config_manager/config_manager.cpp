/**
 * @Author: Yunkai Xia
 * @Date:   2023-09-04 15:56:40
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-09-04 20:35:12
 */
#include "config_manager.h"

#include <iostream>

namespace minco_local_planner::config_manager {
ConfigManager::ConfigManager(const std::string& path)
    : BaseModule("config_manager"), config_file_path_(path) {}

bool ConfigManager::Init() {
  log_cfg_.reset(new LogConfig);
  runtime_manager_cfg_.reset(new RuntimeMangerConfig);
  map_manager_cfg_.reset(new MapManagerConfig);
  safety_manager_cfg_.reset(new SafetyManagerConfig);
  plan_manager_cfg_.reset(new PlanManagerConfig);
  astar_cfg_.reset(new AstarConfig);
  car_like_kino_astar_cfg_.reset(new CarLikeKinoAstarConfig);

  return ParseConfig();
}
bool ConfigManager::Start() { return true; }

void ConfigManager::Stop() {}

bool ConfigManager::ParseConfig() {
  Json config_json;

  try {
    config_json = LoadDeeply(config_file_path_);
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return false;
  }

  if (!ParseLogConfig(config_json["log_cfg"])) {
    std::cout << "failed to parse log_cfg " << std::endl;
    return false;
  }

  if (!ParseRuntimeMangerConfig(config_json["runtime_manager_cfg"])) {
    std::cout << "failed to parse runtime_manager_cfg_json " << std::endl;
    return false;
  }

  if (!ParseMapManagerConfig(config_json["map_manager_cfg"])) {
    std::cout << "failed to parse map_manager_cfg_json " << std::endl;
    return false;
  }

  if (!ParseSafetyManagerConfig(config_json["safety_manager_cfg"])) {
    std::cout << "failed to parse safety_manager_cfg_json " << std::endl;
    return false;
  }

  if (!ParsePlanManagerConfig(config_json["plan_manager_cfg"])) {
    std::cout << "failed to parse plan_manager_cfg_json " << std::endl;
    return false;
  }

  if (!ParseAstarConfig(config_json["astar_cfg"])) {
    std::cout << "failed to parse astar_cfg_json " << std::endl;
    return false;
  }

  return true;
}

bool ConfigManager::ParseLogConfig(const Json& log_cfg_json) {
  if (!GetField(log_cfg_json, "log_level", log_cfg_->log_level)) {
    return false;
  }
  if (!GetField(log_cfg_json, "log_path", log_cfg_->log_path)) {
    return false;
  }

  if (!GetField(log_cfg_json, "log_type", log_cfg_->log_type)) {
    return false;
  }
  return true;
}

bool ConfigManager::ParseRuntimeMangerConfig(
    const Json& runtime_manager_cfg_json) {
  if (!GetField(runtime_manager_cfg_json, "check_sleep_time",
                runtime_manager_cfg_->check_sleep_time)) {
    return false;
  }

  if (!GetField(runtime_manager_cfg_json, "message_wait_time",
                runtime_manager_cfg_->message_wait_time)) {
    return false;
  }
  return true;
}

bool ConfigManager::ParseMapManagerConfig(const Json& map_manager_cfg_json) {
  if (!GetField(map_manager_cfg_json, "base_to_laser_x",
                map_manager_cfg_->base_to_laser_x)) {
    return false;
  }

  if (!GetField(map_manager_cfg_json, "base_to_laser_y",
                map_manager_cfg_->base_to_laser_y)) {
    return false;
  }

  if (!GetField(map_manager_cfg_json, "base_to_laser_yaw",
                map_manager_cfg_->base_to_laser_yaw)) {
    return false;
  }

  if (!GetField(map_manager_cfg_json, "down_sampling_res",
                map_manager_cfg_->down_sampling_res)) {
    return false;
  }

  if (!GetField(map_manager_cfg_json, "grid_map_height",
                map_manager_cfg_->grid_map_height)) {
    return false;
  }

  if (!GetField(map_manager_cfg_json, "grid_map_inf_size",
                map_manager_cfg_->grid_map_inf_size)) {
    return false;
  }

  if (!GetField(map_manager_cfg_json, "grid_map_res",
                map_manager_cfg_->grid_map_res)) {
    return false;
  }

  if (!GetField(map_manager_cfg_json, "grid_map_width",
                map_manager_cfg_->grid_map_width)) {
    return false;
  }

  if (!GetField(map_manager_cfg_json, "laser_max_range",
                map_manager_cfg_->laser_max_range)) {
    return false;
  }

  if (!GetField(map_manager_cfg_json, "laser_min_range",
                map_manager_cfg_->laser_min_range)) {
    return false;
  }

  if (!GetField(map_manager_cfg_json, "map_generate_time",
                map_manager_cfg_->map_generate_time)) {
    return false;
  }

  if (!GetField(map_manager_cfg_json, "pcd_map_path",
                map_manager_cfg_->pcd_map_path)) {
    return false;
  }

  if (!GetField(map_manager_cfg_json, "raycast_dis",
                map_manager_cfg_->raycast_dis)) {
    return false;
  }

  if (!GetField(map_manager_cfg_json, "raycast_res",
                map_manager_cfg_->raycast_res)) {
    return false;
  }

  if (!GetField(map_manager_cfg_json, "use_global_map",
                map_manager_cfg_->use_global_map)) {
    return false;
  }
  return true;
}

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

bool ConfigManager::ParsePlanManagerConfig(const Json& plan_manager_cfg_json) {
  if (!GetField(plan_manager_cfg_json, "path_search_type",
                plan_manager_cfg_->path_search_type)) {
    return false;
  }

  if (!GetField(plan_manager_cfg_json, "plan_sleep_time",
                plan_manager_cfg_->plan_sleep_time)) {
    return false;
  }
  return true;
}

bool ConfigManager::ParseAstarConfig(const Json& astar_cfg_json) {
  if (!GetField(astar_cfg_json, "allocate_num", astar_cfg_->allocate_num)) {
    return false;
  }

  if (!GetField(astar_cfg_json, "heu_type", astar_cfg_->heu_type)) {
    return false;
  }

  if (!GetField(astar_cfg_json, "lambda_heu", astar_cfg_->lambda_heu)) {
    return false;
  }

  if (!GetField(astar_cfg_json, "time_breaker", astar_cfg_->time_breaker)) {
    return false;
  }
  return true;
}
}  // namespace minco_local_planner::config_manager