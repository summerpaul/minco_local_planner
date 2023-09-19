/**
 * @Author: Yunkai Xia
 * @Date:   2023-09-19 10:12:37
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-09-19 10:13:01
 */
#include <stdint.h>

#ifndef __ASTAR_CONFIG_H__
#define __ASTAR_CONFIG_H__

#include <memory>
#include <string>
namespace minco_local_planner::config_manager {

struct AstarConfig {
  typedef std::shared_ptr<AstarConfig> Ptr;
  double lambda_heu = 5;
  int allocate_num = 100000;
  double time_breaker = 1.0001;
  int heu_type = 0;
};
}  // namespace minco_local_planner::config_manager

#endif /* __ASTAR_CONFIG_H__ */
