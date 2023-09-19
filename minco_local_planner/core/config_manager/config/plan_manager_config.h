/**
 * @Author: Yunkai Xia
 * @Date:   2023-09-19 10:10:56
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-09-19 10:11:19
 */
#include <stdint.h>

#ifndef __PLAN_MANAGER_CONFIG_H__
#define __PLAN_MANAGER_CONFIG_H__
#include <memory>
#include <string>
namespace minco_local_planner::config_manager {

struct PlanManagerConfig {
  typedef std::shared_ptr<PlanManagerConfig> Ptr;
  double plan_sleep_time = 0.1;
  int path_search_type = 0;  // 0 :ASTAR, 1:LAZY_THETA_ASTAR,2:KINO_ASTAR
  std::string path_search_lib_path;
  std::string path_search_class_name;
};
}  // namespace minco_local_planner::config_manager

#endif /* __PLAN_MANAGER_CONFIG_H__ */
