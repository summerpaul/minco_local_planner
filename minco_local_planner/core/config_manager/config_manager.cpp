/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-24 15:23:43
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-09-02 10:00:59
 */
#include "config_manager.h"

#include <fstream>
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

  return ParseConfig();
}
bool ConfigManager::Start() { return true; }

void ConfigManager::Stop() {}

bool ConfigManager::ParseConfig() {
  std::ifstream ifs(config_file_path_.data());  // open file example.json
  Json::Value json_cfg;
  Json::Reader reader;

  if (!reader.parse(ifs, json_cfg)) {
    std::cout << "failed to load config file " << std::endl;
    return false;
  }

  std::cout << "json_cfg is " << json_cfg << std::endl;

  if (!ParseLogConfig(json_cfg["log_cfg"])) {
    std::cout << "failed to ParseLogConfig " << std::endl;
    return false;
  }

  if (!ParseRuntimeMangerConfig(json_cfg["runtime_manager_cfg"])) {
    std::cout << "failed to ParseRuntimeMangerConfig " << std::endl;
    return false;
  }
  if (!ParseMapManagerConfig(json_cfg["map_manager_cfg"])) {
    std::cout << "failed to ParseMapManagerConfig " << std::endl;
    return false;
  }
  if (!ParseSafetyManagerConfig(json_cfg["safety_manafger_cfg"])) {
    std::cout << "failed to ParseSafetyManagerConfig " << std::endl;
    return false;
  }

  if (!ParsePlanManagerConfig(json_cfg["plan_manager_cfg"])) {
    std::cout << "failed to ParsePlanManagerConfig " << std::endl;
    return false;
  }

  if (!ParseAstarConfig(json_cfg["astar_cfg"])) {
    std::cout << "failed to ParseAstarConfig " << std::endl;
    return false;
  }

  return true;
}

bool ConfigManager::ParseLogConfig(const Json::Value& log_cfg_json) {
  if (log_cfg_json.type() == Json::nullValue) {
    std::cout << "miss  log config all " << std::endl;
    return false;
  }
  if (log_cfg_json["log_level"].type() == Json::nullValue) {
    std::cout << "miss  log config log_level " << std::endl;
    return false;
  }
  log_cfg_->log_level = log_cfg_json["log_level"].asInt();

  if (log_cfg_json["log_path"].type() == Json::nullValue) {
    std::cout << "miss  log config log_path " << std::endl;
    return false;
  }
  log_cfg_->log_path = log_cfg_json["log_path"].asString();

  if (log_cfg_json["log_type"].type() == Json::nullValue) {
    std::cout << "miss  log config log_type " << std::endl;
    return false;
  }
  log_cfg_->log_type = log_cfg_json["log_type"].asInt();
  return true;
}

bool ConfigManager::ParseRuntimeMangerConfig(
    const Json::Value& runtime_manager_cfg_json) {
  if (runtime_manager_cfg_json.type() == Json::nullValue) {
    std::cout << "miss runtime manager config all " << std::endl;
    return false;
  }
  if (runtime_manager_cfg_json["check_sleep_time"].type() == Json::nullValue) {
    std::cout << "miss runtime manager check_sleep_time  " << std::endl;
    return false;
  }
  runtime_manager_cfg_->check_sleep_time =
      runtime_manager_cfg_json["check_sleep_time"].asDouble();

  if (runtime_manager_cfg_json["message_wait_time"].type() == Json::nullValue) {
    std::cout << "miss runtime manager message_wait_time  " << std::endl;
    return false;
  }
  runtime_manager_cfg_->message_wait_time =
      runtime_manager_cfg_json["message_wait_time"].asDouble();
  return true;
}

bool ConfigManager::ParseMapManagerConfig(
    const Json::Value& map_manager_cfg_json) {
  if (map_manager_cfg_json.type() == Json::nullValue) {
    return false;
  }

  if (map_manager_cfg_json["base_to_laser_x"].type() == Json::nullValue) {
    return false;
  }
  map_manager_cfg_->base_to_laser_x =
      map_manager_cfg_json["base_to_laser_x"].asDouble();

  if (map_manager_cfg_json["base_to_laser_y"].type() == Json::nullValue) {
    return false;
  }
  map_manager_cfg_->base_to_laser_y =
      map_manager_cfg_json["base_to_laser_y"].asDouble();

  if (map_manager_cfg_json["base_to_laser_yaw"].type() == Json::nullValue) {
    return false;
  }
  map_manager_cfg_->base_to_laser_yaw =
      map_manager_cfg_json["base_to_laser_yaw"].asDouble();

  if (map_manager_cfg_json["down_sampling_res"].type() == Json::nullValue) {
    return false;
  }
  map_manager_cfg_->down_sampling_res =
      map_manager_cfg_json["down_sampling_res"].asDouble();

  if (map_manager_cfg_json["grid_map_height"].type() == Json::nullValue) {
    return false;
  }
  map_manager_cfg_->grid_map_height =
      map_manager_cfg_json["grid_map_height"].asDouble();

  if (map_manager_cfg_json["grid_map_width"].type() == Json::nullValue) {
    return false;
  }
  map_manager_cfg_->grid_map_width =
      map_manager_cfg_json["grid_map_width"].asDouble();

  if (map_manager_cfg_json["grid_map_res"].type() == Json::nullValue) {
    return false;
  }
  map_manager_cfg_->grid_map_res =
      map_manager_cfg_json["grid_map_res"].asDouble();

  if (map_manager_cfg_json["grid_map_inf_size"].type() == Json::nullValue) {
    return false;
  }
  map_manager_cfg_->grid_map_inf_size =
      map_manager_cfg_json["grid_map_inf_size"].asDouble();

  if (map_manager_cfg_json["laser_max_range"].type() == Json::nullValue) {
    return false;
  }
  map_manager_cfg_->laser_max_range =
      map_manager_cfg_json["laser_max_range"].asDouble();

  if (map_manager_cfg_json["laser_min_range"].type() == Json::nullValue) {
    return false;
  }
  map_manager_cfg_->laser_min_range =
      map_manager_cfg_json["laser_min_range"].asDouble();

  if (map_manager_cfg_json["map_generate_time"].type() == Json::nullValue) {
    return false;
  }
  map_manager_cfg_->map_generate_time =
      map_manager_cfg_json["map_generate_time"].asDouble();

  if (map_manager_cfg_json["raycast_dis"].type() == Json::nullValue) {
    return false;
  }
  map_manager_cfg_->raycast_dis =
      map_manager_cfg_json["raycast_dis"].asDouble();

  if (map_manager_cfg_json["raycast_res"].type() == Json::nullValue) {
    return false;
  }
  map_manager_cfg_->raycast_res =
      map_manager_cfg_json["raycast_res"].asDouble();

  if (map_manager_cfg_json["pcd_map_path"].type() == Json::nullValue) {
    return false;
  }

  map_manager_cfg_->pcd_map_path =
      map_manager_cfg_json["pcd_map_path"].asString();

  if (map_manager_cfg_json["use_global_map"].type() == Json::nullValue) {
    return false;
  }

  map_manager_cfg_->use_global_map =
      map_manager_cfg_json["use_global_map"].asBool();

  return true;
}

bool ConfigManager::ParseSafetyManagerConfig(
    const Json::Value& safety_manager_cfg_json) {
  if (safety_manager_cfg_json.type() == Json::nullValue) {
    return false;
  }

  if (safety_manager_cfg_json["back_to_center"].type() == Json::nullValue) {
    return false;
  }
  safety_manager_cfg_->back_to_center =
      safety_manager_cfg_json["back_to_center"].asDouble();

  if (safety_manager_cfg_json["creep_box_x_margin"].type() == Json::nullValue) {
    return false;
  }
  safety_manager_cfg_->creep_box_x_margin =
      safety_manager_cfg_json["creep_box_x_margin"].asDouble();

  if (safety_manager_cfg_json["creep_box_y_margin"].type() == Json::nullValue) {
    return false;
  }
  safety_manager_cfg_->creep_box_y_margin =
      safety_manager_cfg_json["creep_box_y_margin"].asDouble();

  if (safety_manager_cfg_json["dangerous_to_safe_counts"].type() ==
      Json::nullValue) {
    return false;
  }
  safety_manager_cfg_->dangerous_to_safe_counts =
      safety_manager_cfg_json["dangerous_to_safe_counts"].asInt();

  if (safety_manager_cfg_json["emergency_stop_length"].type() ==
      Json::nullValue) {
    return false;
  }
  safety_manager_cfg_->emergency_stop_length =
      safety_manager_cfg_json["emergency_stop_length"].asDouble();

  if (safety_manager_cfg_json["max_kappa"].type() == Json::nullValue) {
    return false;
  }
  safety_manager_cfg_->max_kappa =
      safety_manager_cfg_json["max_kappa"].asDouble();

  if (safety_manager_cfg_json["replan_dist"].type() == Json::nullValue) {
    return false;
  }
  safety_manager_cfg_->replan_dist =
      safety_manager_cfg_json["replan_dist"].asDouble();

  if (safety_manager_cfg_json["safe_check_path_length"].type() ==
      Json::nullValue) {
    return false;
  }
  safety_manager_cfg_->safe_check_path_length =
      safety_manager_cfg_json["safe_check_path_length"].asDouble();

  if (safety_manager_cfg_json["safe_check_sleep_time"].type() ==
      Json::nullValue) {
    return false;
  }
  safety_manager_cfg_->safe_check_sleep_time =
      safety_manager_cfg_json["safe_check_sleep_time"].asDouble();

  if (safety_manager_cfg_json["safe_distance"].type() == Json::nullValue) {
    return false;
  }
  safety_manager_cfg_->safe_distance =
      safety_manager_cfg_json["safe_distance"].asDouble();

  if (safety_manager_cfg_json["slow_down_box_x_margin"].type() ==
      Json::nullValue) {
    return false;
  }
  safety_manager_cfg_->slow_down_box_x_margin =
      safety_manager_cfg_json["slow_down_box_x_margin"].asDouble();

  if (safety_manager_cfg_json["slow_down_box_y_margin"].type() ==
      Json::nullValue) {
    return false;
  }
  safety_manager_cfg_->slow_down_box_y_margin =
      safety_manager_cfg_json["slow_down_box_y_margin"].asDouble();

  if (safety_manager_cfg_json["slow_down_length"].type() == Json::nullValue) {
    return false;
  }
  safety_manager_cfg_->slow_down_length =
      safety_manager_cfg_json["slow_down_length"].asDouble();

  if (safety_manager_cfg_json["stop_box_x_margin"].type() == Json::nullValue) {
    return false;
  }
  safety_manager_cfg_->stop_box_x_margin =
      safety_manager_cfg_json["stop_box_x_margin"].asDouble();

  if (safety_manager_cfg_json["stop_box_y_margin"].type() == Json::nullValue) {
    return false;
  }
  safety_manager_cfg_->stop_box_y_margin =
      safety_manager_cfg_json["stop_box_y_margin"].asDouble();

  if (safety_manager_cfg_json["vehicle_length"].type() == Json::nullValue) {
    return false;
  }
  safety_manager_cfg_->vehicle_length =
      safety_manager_cfg_json["vehicle_length"].asDouble();

  if (safety_manager_cfg_json["vehicle_width"].type() == Json::nullValue) {
    return false;
  }
  safety_manager_cfg_->vehicle_width =
      safety_manager_cfg_json["vehicle_width"].asDouble();
  return true;
}

bool ConfigManager::ParsePlanManagerConfig(
    const Json::Value& plan_manager_cfg_json) {
  if (plan_manager_cfg_json.type() == Json::nullValue) {
    return false;
  }

  if (plan_manager_cfg_json["plan_sleep_time"].type() == Json::nullValue) {
    return false;
  }

  plan_manager_cfg_->plan_sleep_time =
      plan_manager_cfg_json["plan_sleep_time"].asDouble();

  if (plan_manager_cfg_json["path_search_type"].type() == Json::nullValue) {
    return false;
  }
  plan_manager_cfg_->path_search_type =
      plan_manager_cfg_json["path_search_type"].asInt();

  return true;
}

bool ConfigManager::ParseAstarConfig(const Json::Value& astar_cfg_json) {
  if (astar_cfg_json.type() == Json::nullValue) {
    return false;
  }

  if (astar_cfg_json["allocate_num"].type() == Json::nullValue) {
    return false;
  }
  astar_cfg_->allocate_num = astar_cfg_json["allocate_num"].asInt();

  if (astar_cfg_json["heu_type"].type() == Json::nullValue) {
    return false;
  }
  astar_cfg_->heu_type = astar_cfg_json["heu_type"].asInt();

  if (astar_cfg_json["lambda_heu"].type() == Json::nullValue) {
    return false;
  }
  astar_cfg_->lambda_heu = astar_cfg_json["lambda_heu"].asDouble();

  if (astar_cfg_json["time_breaker"].type() == Json::nullValue) {
    return false;
  }
  astar_cfg_->time_breaker = astar_cfg_json["time_breaker"].asDouble();
  return true;
}
}  // namespace minco_local_planner::config_manager