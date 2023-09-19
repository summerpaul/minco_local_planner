/**
 * @Author: Yunkai Xia
 * @Date:   2023-09-19 10:08:40
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-09-19 11:19:54
 */
#include <stdint.h>

#ifndef __MAP_MANAGER_CONFIG_H__
#define __MAP_MANAGER_CONFIG_H__

#include <memory>
#include <string>
namespace minco_local_planner::config_manager {

struct MapManagerConfig {
  typedef std::shared_ptr<MapManagerConfig> Ptr;
  std::string pcd_map_path = "/root/locate/laser_kc.pcd";  // 点云文件地址
  double map_generate_time = 0.1;   // 地图生成时间间隔
  double down_sampling_res = 0.05;  // 点云数据降采样分辨率 单位m
  double grid_map_res = 0.05;       // 栅格地图的分辨率 单位m
  double grid_map_inf_size = 0.2;   // 栅格地图的膨胀半径 单位m
  double raycast_dis = 0.5;      // 实时点云后，障碍物强厚度单位m
  double raycast_res = 0.1;      // 增加点的距离分辨率 单位m
  double laser_max_range = 5.0;  // 激光雷达最大的范围 单位m
  double laser_min_range = 0.1;  // 激光雷达最小的范围 单位m
  double base_to_laser_x = 0;  // 激光雷达与车体坐标系的相对坐标x
  double base_to_laser_y = 0;  // 激光雷达与车体坐标系的相对坐标y
  double base_to_laser_yaw = 0;  // 激光雷达与车体坐标系的相对坐标yaw
  double local_map_width = 4.0;   // 地图的宽 单位 m(车身轴向)
  double local_map_height = 7.0;  // 地图的高 单位m（车身横向）
  bool use_global_map = false;    // 使用启用全局地图
  double esdf_map_res = 0.1;
  bool use_esdf_map = false;
  double width_offset_scale = -0.5;
  double height_offset_scale = -0.5;
};
}  // namespace minco_local_planner::config_manager

#endif /* __MAP_MANAGER_CONFIG_H__ */
