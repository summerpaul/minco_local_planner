/**
 * @Author: Yunkai Xia
 * @Date:   2023-09-19 10:10:00
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-09-19 10:10:22
 */
#include <stdint.h>

#ifndef __SAFETY_MANAGER_CONFIG_H__
#define __SAFETY_MANAGER_CONFIG_H__

#include <memory>
#include <string>
namespace minco_local_planner::config_manager {
// 安全管理配置
struct SafetyManagerConfig {
  typedef std::shared_ptr<SafetyManagerConfig> Ptr;
  double safe_check_path_length = 3.0;  //   安全检车路径的长度
  double safe_check_sleep_time = 0.1;   //   检查安全路径的时间间隔
  double emergency_stop_length = 0.5;   //   紧急停止的路径长度
  double safe_distance = 0.2;           //   安全距离
  double replan_dist = 0.1;             // 优化的最短距离
  double slow_down_length = 1.0;        //   减速行驶的路径长度
  double max_kappa = 100;               // 轨迹的最大曲率
  double vehicle_length = 0.5;          // 增加车辆轮廓
  double vehicle_width = 0.5;
  double back_to_center = 0.25;
  double slow_down_box_x_margin = 0.5;
  double slow_down_box_y_margin = 0.5;
  double stop_box_x_margin = 0.3;
  double stop_box_y_margin = 0.3;
  double creep_box_x_margin = 0.1;
  double creep_box_y_margin = 0.1;
  int dangerous_to_safe_counts = 20;  // 车辆从危险状态转到安全状态counts
};

}  // namespace minco_local_planner::config_manager

#endif /* __SAFETY_MANAGER_CONFIG_H__ */
