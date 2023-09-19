/**
 * @Author: Yunkai Xia
 * @Date:   2023-09-19 11:08:37
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-09-19 11:09:24
 */
#include <iostream>

#include "config_manager.h"
namespace minco_local_planner::config_manager {
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
}  // namespace minco_local_planner::config_manager