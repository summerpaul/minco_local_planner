/**
 * @Author: Xia Yunkai
 * @Date:   2023-08-24 21:22:24
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-08-30 23:03:05
 */
#include "map_manager.h"

#include <iostream>

#include "basis/time.h"
#include "module_manager/module_manager.h"
#include "utils/pcl_tools.h"
#include "utils/singleton.h"
#include "utils/timer_manager.h"
namespace minco_local_planner::map_manager {

MapManager::MapManager() : BaseModule("MapManager") {}
MapManager::~MapManager() {}
using namespace module_manager;
bool MapManager::Init() {
  cfg_ =
      ModuleManager::GetInstance()->GetConfigManager()->GetMapManagerConfig();
  GenerateInitLocalGridMap();
  base_to_laser_[0] = cfg_.base_to_laser_x;
  base_to_laser_[1] = cfg_.base_to_laser_y;
  base_to_laser_[2] = cfg_.base_to_laser_yaw;

  if (cfg_.use_global_map) {
    if (!GenerateInitGlobalGridMap()) {
      LOG_ERROR("failed to load pcd map");
      return false;
    }
    b_have_global_map_ = true;
  }

  return true;
}
bool MapManager::Start() {
  Singleton<TimerManager>()->Schedule(
      int(cfg_.map_generate_time * 1000),
      std::bind(&MapManager::GenerateTransformedPointcloudTimer, this));
  Singleton<TimerManager>()->Schedule(
      int(cfg_.map_generate_time * 1000),
      std::bind(&MapManager::GenerateLocalGridMapTimer, this));

  if (cfg_.use_global_map) {
    Singleton<TimerManager>()->Schedule(
        int(cfg_.map_generate_time * 1000),
        std::bind(&MapManager::GenerateGlobalGridMapTimer, this));
  }
  return true;
}
void MapManager::Stop() {}

void MapManager::GenerateInitLocalGridMap() {
  local_map_ptr_.reset(new GridMap("local_grid_map"));
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
  local_map_ptr_->CreateGridMap(origin, dim, data, res);
  std::cout << "dim is " << dim << std::endl;
  b_have_local_map_ = true;
}

bool MapManager::GenerateInitGlobalGridMap() {
  PointCloud3d cloud;
  if (!LoadPointCloud(cfg_.pcd_map_path, cloud)) {
    LOG_ERROR("failed to load pcd map ");
    return false;
  }
  downSampling(cloud, cfg_.grid_map_res);
  Pose2d origin;
  Vec2i dim;
  GenerateGridMap(cloud, origin, dim, global_map_data_, cfg_.grid_map_res,
                  cfg_.grid_map_inf_size);

  global_map_ptr_.reset(new GridMap("global_grid_map"));
  global_map_ptr_->CreateGridMap(origin, dim, global_map_data_,
                                 cfg_.grid_map_res);
  LOG_INFO("points size is {}", cloud.size());
  LOG_INFO("global_map_data_ size is {}", global_map_data_.size());

  return true;
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
void MapManager::GenerateTransformedPointcloudTimer() {
  std::lock_guard<std::mutex> lock(transformed_pointcloud_mutex_);
  const LaserScan scan =
      ModuleManager::GetInstance()->GetRuntimeManager()->GetLaserScan();
  PointCloud3d local_point_cloud;

  raycast(scan, local_point_cloud);
  TransformPointCloud(local_point_cloud, base_to_laser_,
                      transformed_pointcloud_);
}

void MapManager::GenerateLocalGridMapTimer() {
  // 获取原始的lasercan数据

  std::lock_guard<std::mutex> lock(transformed_pointcloud_mutex_);
  const VehiclePose pose =
      ModuleManager::GetInstance()->GetRuntimeManager()->GetVehiclePose();

  // 重置栅格地图数据
  local_map_ptr_->SetDataZero();
  //
  local_map_ptr_->SetOrigin(AddPose2d(pose.GetPose2d(), map_offset_));

  for (auto &point : transformed_pointcloud_.points) {
    const int x =
        std::round(point.x * res_inv_ - 0.5) + std::ceil(0.5 * width_);
    const int y =
        std::round(point.y * res_inv_ - 0.5) + std::ceil(0.5 * height_);
    Vec2d pt = local_map_ptr_->IntToDouble(Vec2i(x, y));
    local_map_ptr_->SetInfOccupied(pt, cfg_.grid_map_inf_size);
  }
}

void MapManager::GenerateGlobalGridMapTimer() {
  std::lock_guard<std::mutex> lock(transformed_pointcloud_mutex_);
  const Pose2d origin = ModuleManager::GetInstance()
                            ->GetRuntimeManager()
                            ->GetVehiclePose()
                            .GetPose2d();

  global_map_ptr_->SetData(global_map_data_);
  for (auto &point : transformed_pointcloud_.points) {
    Vec2d inr(point.x, point.y);
    Vec2d pt = AddVec2d(origin, inr);
    global_map_ptr_->SetInfOccupied(pt, cfg_.grid_map_inf_size);
  }
}

}  // namespace minco_local_planner::map_manager