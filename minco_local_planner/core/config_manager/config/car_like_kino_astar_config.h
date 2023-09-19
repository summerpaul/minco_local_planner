/**
 * @Author: Yunkai Xia
 * @Date:   2023-09-19 10:14:15
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-09-19 10:14:39
 */
#include <stdint.h>

#ifndef __CAR_LIKE_KINO_ASTAR_CONFIG_H__
#define __CAR_LIKE_KINO_ASTAR_CONFIG_H__

#include <memory>
#include <string>
namespace minco_local_planner::config_manager {
struct CarLikeKinoAstarConfig {
  typedef std::shared_ptr<CarLikeKinoAstarConfig> Ptr;
  double step_arc = 1.0;
  double max_vel = 10.0;
  double max_acc = 3.0;
  double min_vel = -1.0;
  double min_acc = -1.0;
  double max_cur = 0.35;
  double max_steer = 0.78539815;
  double max_seach_time = 0.1;
  double traj_forward_penalty = 1.0;
  double traj_back_penalty = 1.0;
  double traj_gear_switch_penalty = 10.0;
  double traj_steer_penalty = 0.0;
  double traj_steer_change_penalty = 0.0;
  double horizon = 50;
  double lambda_heu = 5.0;
  double time_resolution = 0.1;
  double distance_resolution = 0.5;
  double velocity_resolution = 0.5;
  int allocate_num = 100000;
  int check_num = 5;
};

}  // namespace minco_local_planner::config_manager

#endif /* __CAR_LIKE_KINO_ASTAR_CONFIG_H__ */
