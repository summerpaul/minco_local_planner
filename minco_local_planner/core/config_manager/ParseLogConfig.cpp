/**
 * @Author: Yunkai Xia
 * @Date:   2023-09-19 11:07:29
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-09-19 11:08:28
 */
#include <iostream>

#include "config_manager.h"
namespace minco_local_planner::config_manager {
    
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
}  // namespace minco_local_planner::config_manager