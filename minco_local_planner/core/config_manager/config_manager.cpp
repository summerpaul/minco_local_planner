/**
 * @Author: Yunkai Xia
 * @Date:   2023-09-04 15:56:40
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-09-19 11:12:14
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
}  // namespace minco_local_planner::config_manager