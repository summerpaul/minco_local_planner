/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-24 15:11:34
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-09-03 04:17:43
 */
#include <stdint.h>

#ifndef __CONFIG_DATA_H__
#define __CONFIG_DATA_H__
#include <memory>
#include <string>
namespace minco_local_planner::config_manager {
struct LogConfig {
  typedef std::shared_ptr<LogConfig> Ptr;
  std::string log_path = "/root/log";  // 日志文件目录
  int log_type = 0;   // 日志类型，0：console_only 1:file_only 2:both
  int log_level = 0;  // 日志等级
};

struct RuntimeMangerConfig {
  typedef std::shared_ptr<RuntimeMangerConfig> Ptr;
  double check_sleep_time = 0.1;   // 检查数据异常的频率
  double message_wait_time = 0.5;  //   数据异常的时间间隔
};

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
  double grid_map_width = 4.0;   // 地图的宽 单位 m(车身轴向)
  double grid_map_height = 7.0;  // 地图的高 单位m（车身横向）
  bool use_global_map = false;   // 使用启用全局地图
};
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

struct PlanManagerConfig {
  typedef std::shared_ptr<PlanManagerConfig> Ptr;
  double plan_sleep_time = 0.1;
  int path_search_type = 0;  // 0 :ASTAR, 1:LAZY_THETA_ASTAR,2:KINO_ASTAR
};

struct AstarConfig {
  typedef std::shared_ptr<AstarConfig> Ptr;
  double lambda_heu = 5;
  int allocate_num = 100000;
  double time_breaker = 1.0001;
  int heu_type = 0;
};

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

struct KinoAstarConfig {
  typedef std::shared_ptr<KinoAstarConfig> Ptr;
  double max_vel = 0.8;
  double max_acc = 1.0;
  double max_tau = 0.5;
  int allocate_num = 100000;
  int check_num = 5;
  double lambda_heu = 5.0;
};

}  // namespace minco_local_planner::config_manager

#endif /* __CONFIG_DATA_H__ */
