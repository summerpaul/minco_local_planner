/**
 * @Author: Xia Yunkai
 * @Date:   2023-08-24 21:22:19
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-08-28 22:19:17
 */
#include <stdint.h>

#ifndef __MAP_MANAGER_H__
#define __MAP_MANAGER_H__
#include <mutex>

#include "basis/base_module.h"
#include "basis/laser_scan.h"
#include "basis/logger.h"
#include "config_manager/config_data.h"
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

  const GridMap::Ptr GetGridMap() const { return grid_map_ptr_; }

  const PointCloud3d &GetTransformedPointcloud() const {
    return transformed_pointcloud_;
  }

 private:
  void GenerateGridMapTimer();

  void GenerateInitGridMap();

 private:
  void raycast(const LaserScan &laser_scan_in, PointCloud3d &cloud_out);

 private:
  GridMap::Ptr grid_map_ptr_;  // 车身坐标系下的局部栅格地图
  MapManagerConfig cfg_;
  std::mutex transformed_pointcloud_mutex_;
  PointCloud3d transformed_pointcloud_;  // 使用外参标定后的点云地图，用于停障
  Pose2d base_to_laser_;
  Pose2d map_offset_;
  double res_inv_;
  int width_;
  int height_;

};

}  // namespace minco_local_planner::map_manager

#endif /* __MAP_MANAGER_H__ */
