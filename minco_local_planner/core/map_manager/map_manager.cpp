/**
 * @Author: Xia Yunkai
 * @Date:   2023-08-24 21:22:24
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-08-30 13:20:47
 */
#include "map_manager.h"

#include <iostream>

#include "basis/time.h"
#include "module_manager/module_manager.h"
#include "utils/singleton.h"
#include "utils/timer_manager.h"
namespace minco_local_planner::map_manager {

MapManager::MapManager() : BaseModule("MapManager") {}
MapManager::~MapManager() {}
using namespace module_manager;
bool MapManager::Init() {
  cfg_ =
      ModuleManager::GetInstance()->GetConfigManager()->GetMapManagerConfig();
  GenerateInitGridMap();
  base_to_laser_[0] = cfg_.base_to_laser_x;
  base_to_laser_[1] = cfg_.base_to_laser_y;
  base_to_laser_[2] = cfg_.base_to_laser_yaw;

  return true;
}
bool MapManager::Start() {
  Singleton<TimerManager>()->Schedule(
      int(cfg_.map_generate_time * 1000),
      std::bind(&MapManager::GenerateGridMapTimer, this));
  return true;
}
void MapManager::Stop() {}

void MapManager::GenerateInitGridMap() {
  grid_map_ptr_.reset(new GridMap);

  const double res = cfg_.grid_map_res;

  res_inv_ = 1 / cfg_.grid_map_res;
  width_ = std::ceil(cfg_.grid_map_width * res_inv_);
  height_ = std::ceil(cfg_.grid_map_height * res_inv_);
  std::vector<int8_t> data;
  const int data_size = width_ * height_;

  data.resize(data_size);
  data.assign(data_size, FREE);
  map_offset_ = -0.5 * Pose2d(cfg_.grid_map_width, cfg_.grid_map_height, 0);

  Pose2d origin;

  const Vec2i dim(width_, height_);

  grid_map_ptr_->CreateGridMap(origin, dim, data, res);
}

void MapManager::raycast(const LaserScan &laser_scan_in, PointCloud3d &cloud) {
  cloud.points.clear();
  for (size_t i = 0; i < laser_scan_in.ranges.size(); ++i) {
    const double cur_angle =
        laser_scan_in.angle_min + i * laser_scan_in.angle_increment;
    const double cur_range = laser_scan_in.ranges[i];
    const double cur_angle_cos = std::cos(cur_angle);
    const double cur_angle_sin = std::sin(cur_angle);
    // 根据配置的激光雷达范围获取点云
    if (cur_range < cfg_.laser_max_range && cur_range > cfg_.laser_min_range) {
      cloud.points.emplace_back(pcl::PointXYZ(cur_angle_cos * cur_range,
                                              cur_angle_sin * cur_range, 0));
      for (double raycast_range = cur_range + cfg_.raycast_res;
           raycast_range < cur_range + cfg_.raycast_dis;
           raycast_range += cfg_.raycast_res) {
        cloud.points.emplace_back(pcl::PointXYZ(
            cur_angle_cos * raycast_range, cur_angle_sin * raycast_range, 0));
      }
    }
  }
  cloud.width = cloud.points.size();
  cloud.height = 1;
}

void MapManager::GenerateGridMapTimer() {
  // 获取原始的lasercan数据
  const LaserScan scan =
      ModuleManager::GetInstance()->GetRuntimeManager()->GetLaserScan();
  PointCloud3d local_point_cloud;

  raycast(scan, local_point_cloud);
  TransformPointCloud(local_point_cloud, base_to_laser_,
                      transformed_pointcloud_);
  const VehiclePose pose =
      ModuleManager::GetInstance()->GetRuntimeManager()->GetVehiclePose();

  const size_t points_size = transformed_pointcloud_.points.size();
  // 重置栅格地图数据
  grid_map_ptr_->SetDataZero();
  //
  grid_map_ptr_->SetOrigin(AddPose2d(pose.GetPose2d(), map_offset_));

  for (size_t i = 0; i < points_size; ++i) {
    const int x = std::ceil(transformed_pointcloud_.points[i].x * res_inv_) +
                  std::ceil(0.5 * width_);
    const int y = std::ceil(transformed_pointcloud_.points[i].y * res_inv_) +
                  std::ceil(0.5 * height_);
    grid_map_ptr_->SetOccupied(Vec2i(x, y));
  }
}

}  // namespace minco_local_planner::map_manager