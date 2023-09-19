/**
 * @Author: Yunkai Xia
 * @Date:   2023-09-19 10:13:27
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-09-19 10:13:54
 */
#include <stdint.h>

#ifndef __KINO_ASTAR_CONFIG_H__
#define __KINO_ASTAR_CONFIG_H__

#include <memory>
#include <string>
namespace minco_local_planner::config_manager {
struct KinoAstarConfig {
  typedef std::shared_ptr<KinoAstarConfig> Ptr;
  double max_vel = 0.8;
  double max_acc = 1.0;
  double max_tau = 0.5;
  int allocate_num = 100000;
  int check_num = 5;
  double lambda_heu = 5.0;
  double max_search_time = 2.0;
};
}  // namespace minco_local_planner::config_manager

#endif /* __KINO_ASTAR_CONFIG_H__ */
