/**
 * @Author: Yunkai Xia
 * @Date:   2023-09-19 11:11:44
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-09-19 11:12:03
 */
#include "config_manager.h"
namespace minco_local_planner::config_manager {

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