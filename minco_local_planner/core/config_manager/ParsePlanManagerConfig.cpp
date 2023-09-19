/**
 * @Author: Yunkai Xia
 * @Date:   2023-09-19 11:11:10
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-09-19 11:11:35
 */

#include "config_manager.h"
namespace minco_local_planner::config_manager {

bool ConfigManager::ParsePlanManagerConfig(const Json& plan_manager_cfg_json) {
  if (!GetField(plan_manager_cfg_json, "path_search_type",
                plan_manager_cfg_->path_search_type)) {
    return false;
  }

  if (!GetField(plan_manager_cfg_json, "plan_sleep_time",
                plan_manager_cfg_->plan_sleep_time)) {
    return false;
  }

  if (!GetField(plan_manager_cfg_json, "path_search_class_name",
                plan_manager_cfg_->path_search_class_name)) {
    return false;
  }

  if (!GetField(plan_manager_cfg_json, "path_search_lib_path",
                plan_manager_cfg_->path_search_lib_path)) {
    return false;
  }
  return true;
}
}  // namespace minco_local_planner::config_manager