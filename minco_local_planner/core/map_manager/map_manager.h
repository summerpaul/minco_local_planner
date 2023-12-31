/**
 * @Author: Xia Yunkai
 * @Date:   2023-08-24 21:22:19
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-09-05 09:50:34
 */
#include <stdint.h>

#ifndef __MAP_MANAGER_H__
#define __MAP_MANAGER_H__
#include <mutex>

#include "basis/base_module.h"
#include "basis/laser_scan.h"
#include "basis/logger.h"
#include "config_manager/config_data.h"
#include "esdf_map.h"
#include "grid_map.h"
#include "utils/pcl_tools.h"
namespace minco_local_planner::map_manager {

using namespace basis;
using namespace utils;
using namespace config_manager;
class MapManager : public BaseModule {
 public:
  typedef std::shared_ptr<MapManager> Ptr;

 public:
  MapManager();
  ~MapManager();

  virtual bool Init() override;
  virtual bool Start() override;
  virtual void Stop() override;

  const GridMap::Ptr GetLocalMap() const { return local_map_ptr_; }

  const GridMap::Ptr GetGlobalMap() const { return global_map_ptr_; }

  const PointCloud3d &GetTransformedPointcloud() const {
    return transformed_pointcloud_;
  }
  const bool HaveLocalMap() const { return b_have_local_map_; }

  const bool HaveGlobalMap() const { return b_have_global_map_; }

  const bool HaveESDFMap() const { return b_have_esdf_map_; }

  void GetESDFPointCloud(PointCloud3di &cloud);

 private:
  //  生成车体坐标系下的点云的定时器
  void GenerateTransformedPointcloudTimer();
  // 生成局部地图的定时器
  void GenerateLocalGridMapTimer();
  // 生成全局地图的计时器
  void GenerateGlobalGridMapTimer();

  void GenerateLocalESDFMapTimer();
  // 生成初始的局部栅格地图
  void GenerateInitLocalGridMap();
  // 生成初始的全局地图
  bool GenerateInitGlobalGridMap();
  // 生成初始的esdf地图
  void GenerateInitLocalESDFMap();

 private:
  void raycast(const LaserScan &laser_scan_in, PointCloud3d &cloud_out);

 private:
  GridMap::Ptr local_map_ptr_;   // 局部地图
  GridMap::Ptr global_map_ptr_;  // 全局地图
  ESDFMap::Ptr esdf_map_ptr_;
  bool b_have_local_map_ = false;
  bool b_have_global_map_ = false;
  bool b_have_esdf_map_ = false;
  std::vector<int8_t> global_map_data_;  // 用于存储静态地图的数据
  MapManagerConfig::Ptr cfg_;
  std::mutex transformed_pointcloud_mutex_;
  PointCloud3d transformed_pointcloud_;  // 使用外参标定后的点云地图，用于停障
  Pose2d base_to_laser_;
  Pose2d map_offset_;
  double grid_map_res_inv_;
  double esdf_map_res_inv_;
  int grid_map_width_;
  int grid_map_height_;
  int esdf_map_width_;
  int esdf_map_height_;
};

}  // namespace minco_local_planner::map_manager

#endif /* __MAP_MANAGER_H__ */
