/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-24 15:11:34
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-08-28 00:07:16
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

}  // namespace minco_local_planner::config_manager

#endif /* __CONFIG_DATA_H__ */
