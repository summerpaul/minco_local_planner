/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-24 15:23:43
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-08-30 13:30:24
 */
#include "config_manager.h"

#include <fstream>
#include <iostream>

namespace minco_local_planner::config_manager {

ConfigManager::ConfigManager(const std::string& path)
    : BaseModule("config_manager"), config_file_path_(path) {}

bool ConfigManager::Init() {
  std::ifstream ifs(config_file_path_.data());  // open file example.json
  Json::Value json_cfg;
  Json::Reader reader;
  if (!reader.parse(ifs, json_cfg)) {
    std::cout << "failed to load config file " << std::endl;
    return false;
  }

  bool flag = ParseLogConfig(json_cfg["log_cfg"]);
  flag = ParseRuntimeMangerConfig(json_cfg["runtime_manager_cfg"]);
  flag = ParseMapManagerConfig(json_cfg["map_manager_cfg"]);
  flag = ParseSafetyManagerConfig(json_cfg["safety_manafger_cfg"]);
  return flag;
}
bool ConfigManager::Start() { return true; }

void ConfigManager::Stop() {}

bool ConfigManager::ParseLogConfig(const Json::Value& log_cfg_json) {
  if (log_cfg_json.type() == Json::nullValue) {
    std::cout << "miss  log config all " << std::endl;
    return false;
  }
  if (log_cfg_json["log_level"].type() == Json::nullValue) {
    std::cout << "miss  log config log_level " << std::endl;
    return false;
  }
  log_cfg_.log_level = log_cfg_json["log_level"].asInt();

  if (log_cfg_json["log_path"].type() == Json::nullValue) {
    std::cout << "miss  log config log_path " << std::endl;
    return false;
  }
  log_cfg_.log_path = log_cfg_json["log_path"].asString();

  if (log_cfg_json["log_type"].type() == Json::nullValue) {
    std::cout << "miss  log config log_type " << std::endl;
    return false;
  }
  log_cfg_.log_type = log_cfg_json["log_type"].asInt();
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
  runtime_manager_cfg_.check_sleep_time =
      runtime_manager_cfg_json["check_sleep_time"].asDouble();

  if (runtime_manager_cfg_json["message_wait_time"].type() == Json::nullValue) {
    std::cout << "miss runtime manager message_wait_time  " << std::endl;
    return false;
  }
  runtime_manager_cfg_.message_wait_time =
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
  map_manager_cfg_.base_to_laser_x =
      map_manager_cfg_json["base_to_laser_x"].asDouble();

  if (map_manager_cfg_json["base_to_laser_y"].type() == Json::nullValue) {
    return false;
  }
  map_manager_cfg_.base_to_laser_y =
      map_manager_cfg_json["base_to_laser_y"].asDouble();

  if (map_manager_cfg_json["base_to_laser_yaw"].type() == Json::nullValue) {
    return false;
  }
  map_manager_cfg_.base_to_laser_yaw =
      map_manager_cfg_json["base_to_laser_yaw"].asDouble();

  if (map_manager_cfg_json["down_sampling_res"].type() == Json::nullValue) {
    return false;
  }
  map_manager_cfg_.down_sampling_res =
      map_manager_cfg_json["down_sampling_res"].asDouble();

  if (map_manager_cfg_json["grid_map_height"].type() == Json::nullValue) {
    return false;
  }
  map_manager_cfg_.grid_map_height =
      map_manager_cfg_json["grid_map_height"].asDouble();

  if (map_manager_cfg_json["grid_map_width"].type() == Json::nullValue) {
    return false;
  }
  map_manager_cfg_.grid_map_width =
      map_manager_cfg_json["grid_map_width"].asDouble();

  if (map_manager_cfg_json["grid_map_res"].type() == Json::nullValue) {
    return false;
  }
  map_manager_cfg_.grid_map_res =
      map_manager_cfg_json["grid_map_res"].asDouble();

  if (map_manager_cfg_json["grid_map_inf_size"].type() == Json::nullValue) {
    return false;
  }
  map_manager_cfg_.grid_map_inf_size =
      map_manager_cfg_json["grid_map_inf_size"].asDouble();

  if (map_manager_cfg_json["laser_max_range"].type() == Json::nullValue) {
    return false;
  }
  map_manager_cfg_.laser_max_range =
      map_manager_cfg_json["laser_max_range"].asDouble();

  if (map_manager_cfg_json["laser_min_range"].type() == Json::nullValue) {
    return false;
  }
  map_manager_cfg_.laser_min_range =
      map_manager_cfg_json["laser_min_range"].asDouble();

  if (map_manager_cfg_json["map_generate_time"].type() == Json::nullValue) {
    return false;
  }
  map_manager_cfg_.map_generate_time =
      map_manager_cfg_json["map_generate_time"].asDouble();

  if (map_manager_cfg_json["raycast_dis"].type() == Json::nullValue) {
    return false;
  }
  map_manager_cfg_.raycast_dis = map_manager_cfg_json["raycast_dis"].asDouble();

  if (map_manager_cfg_json["raycast_res"].type() == Json::nullValue) {
    return false;
  }
  map_manager_cfg_.raycast_res = map_manager_cfg_json["raycast_res"].asDouble();
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
  safety_manager_cfg_.back_to_center =
      safety_manager_cfg_json["back_to_center"].asDouble();

  if (safety_manager_cfg_json["creep_box_x_margin"].type() == Json::nullValue) {
    return false;
  }
  safety_manager_cfg_.creep_box_x_margin =
      safety_manager_cfg_json["creep_box_x_margin"].asDouble();

  if (safety_manager_cfg_json["creep_box_y_margin"].type() == Json::nullValue) {
    return false;
  }
  safety_manager_cfg_.creep_box_y_margin =
      safety_manager_cfg_json["creep_box_y_margin"].asDouble();

  if (safety_manager_cfg_json["dangerous_to_safe_counts"].type() ==
      Json::nullValue) {
    return false;
  }
  safety_manager_cfg_.dangerous_to_safe_counts =
      safety_manager_cfg_json["dangerous_to_safe_counts"].asInt();

  if (safety_manager_cfg_json["emergency_stop_length"].type() ==
      Json::nullValue) {
    return false;
  }
  safety_manager_cfg_.emergency_stop_length =
      safety_manager_cfg_json["emergency_stop_length"].asDouble();

  if (safety_manager_cfg_json["max_kappa"].type() == Json::nullValue) {
    return false;
  }
  safety_manager_cfg_.max_kappa =
      safety_manager_cfg_json["max_kappa"].asDouble();

  if (safety_manager_cfg_json["replan_dist"].type() == Json::nullValue) {
    return false;
  }
  safety_manager_cfg_.replan_dist =
      safety_manager_cfg_json["replan_dist"].asDouble();

  if (safety_manager_cfg_json["safe_check_path_length"].type() ==
      Json::nullValue) {
    return false;
  }
  safety_manager_cfg_.safe_check_path_length =
      safety_manager_cfg_json["safe_check_path_length"].asDouble();

  if (safety_manager_cfg_json["safe_check_sleep_time"].type() ==
      Json::nullValue) {
    return false;
  }
  safety_manager_cfg_.safe_check_sleep_time =
      safety_manager_cfg_json["safe_check_sleep_time"].asDouble();

  if (safety_manager_cfg_json["safe_distance"].type() == Json::nullValue) {
    return false;
  }
  safety_manager_cfg_.safe_distance =
      safety_manager_cfg_json["safe_distance"].asDouble();

  if (safety_manager_cfg_json["slow_down_box_x_margin"].type() ==
      Json::nullValue) {
    return false;
  }
  safety_manager_cfg_.slow_down_box_x_margin =
      safety_manager_cfg_json["slow_down_box_x_margin"].asDouble();

  if (safety_manager_cfg_json["slow_down_box_y_margin"].type() ==
      Json::nullValue) {
    return false;
  }
  safety_manager_cfg_.slow_down_box_y_margin =
      safety_manager_cfg_json["slow_down_box_y_margin"].asDouble();

  if (safety_manager_cfg_json["slow_down_length"].type() == Json::nullValue) {
    return false;
  }
  safety_manager_cfg_.slow_down_length =
      safety_manager_cfg_json["slow_down_length"].asDouble();

  if (safety_manager_cfg_json["stop_box_x_margin"].type() == Json::nullValue) {
    return false;
  }
  safety_manager_cfg_.stop_box_x_margin =
      safety_manager_cfg_json["stop_box_x_margin"].asDouble();

  if (safety_manager_cfg_json["stop_box_y_margin"].type() == Json::nullValue) {
    return false;
  }
  safety_manager_cfg_.stop_box_y_margin =
      safety_manager_cfg_json["stop_box_y_margin"].asDouble();

  if (safety_manager_cfg_json["vehicle_length"].type() == Json::nullValue) {
    return false;
  }
  safety_manager_cfg_.vehicle_length =
      safety_manager_cfg_json["vehicle_length"].asDouble();

  if (safety_manager_cfg_json["vehicle_width"].type() == Json::nullValue) {
    return false;
  }
  safety_manager_cfg_.vehicle_width =
      safety_manager_cfg_json["vehicle_width"].asDouble();
  return true;
}
}  // namespace minco_local_planner::config_manager