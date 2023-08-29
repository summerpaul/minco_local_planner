/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-24 15:11:34
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-08-29 18:49:12
 */
#include <stdint.h>

#ifndef __CONFIG_DATA_H__
#define __CONFIG_DATA_H__
#include <string>

namespace minco_local_planner::config_manager {
struct LogConfig {
  std::string log_path = "/root/log";
  int log_type = 0;
  int log_level = 0;
};

struct RuntimeMangerConfig {
  double check_sleep_time = 0.1;  // 检查数据异常的频率

  double message_wait_time = 0.5;  //   数据异常的时间间隔
};

struct MapManagerConfig {
  double map_generate_time =
      0.1;  // 接受点云数据的时间间隔，如果时间间隔过长，人为数据异常单位s

  double down_sampling_res = 0.05;  // 点云数据降采样分辨率 单位m

  double grid_map_res = 0.05;  // 栅格地图的分辨率 单位m

  double grid_map_inf_size = 0.2;  // 栅格地图的膨胀半径 单位m

  double raycast_dis = 0.5;  // 实时点云后，障碍物强厚度单位m

  double raycast_res = 0.1;  // 增加点的距离分辨率 单位m

  double laser_max_range = 5.0;  // 激光雷达的范围 单位m

  double laser_min_range = 0.1;

  double base_to_laser_x = 0;  // 激光雷达与车体坐标系的相对坐标
  double base_to_laser_y = 0;
  double base_to_laser_yaw = 0;

  double grid_map_width = 10.0;  // 地图的宽 单位 m

  double grid_map_height = 10.0;  // 地图的高 单位m
};
// 安全管理配置
struct SafetyManagerConfig {
  //   安全检车路径的长度
  double safe_check_path_length{3.0};
  //   检查安全路径的时间减肥
  double safe_check_sleep_time{0.1};
  //   紧急停止的路径长度
  double emergency_stop_length{0.5};
  //   安全距离
  double safe_distance{0.2};
  // 优化的最短距离
  double replan_dist{0.1};

  double slow_down_length{1.0};
  //
  double max_kappa{100};
  // 增加车辆轮廓
  double vehicle_length{0.5};
  double vehicle_width{0.5};
  double back_to_center{0.25};
  double slow_down_box_x_margin{0.5};
  double slow_down_box_y_margin{0.5};
  double stop_box_x_margin{0.3};
  double stop_box_y_margin{0.3};
  double creep_box_x_margin{0.1};
  double creep_box_y_margin{0.1};

  int dangerous_to_safe_counts{20};  // 车辆从危险状态转到安全状态counts
};
}  // namespace minco_local_planner::config_manager

#endif /* __CONFIG_DATA_H__ */
