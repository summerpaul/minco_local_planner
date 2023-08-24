/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-24 15:23:43
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-08-24 16:43:35
 */
#include "config_manager.h"

#include <fstream>
#include <iostream>

namespace minco_local_planner::config_manager {

ConfigManager::ConfigManager(const std::string& path)
    : config_file_path_(path), BaseModule("config_manager") {}

bool ConfigManager::Init() {
  std::ifstream ifs(config_file_path_.data());  // open file example.json
  Json::Value json_cfg;
  Json::Reader reader;
  if (!reader.parse(ifs, json_cfg)) {
    std::cout << "failed to load config file " << std::endl;
    return false;
  }

  bool flag = ParseLogConfig(json_cfg["log_cfg"]);

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
}  // namespace minco_local_planner::config_manager