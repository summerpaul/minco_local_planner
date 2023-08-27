/**
 * @Author: Xia Yunkai
 * @Date:   2023-08-24 21:22:19
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-08-27 23:04:24
 */
#include <stdint.h>

#ifndef __MAP_MANAGER_H__
#define __MAP_MANAGER_H__
#include "basis/base_module.h"
#include "basis/logger.h"
#include "grid_map.h"
#include "utils/pcl_tools.h"
namespace minco_local_planner::map_manager {

using namespace basis;
using namespace utils;
class MapManager : public BaseModule {
 public:
  typedef std::shared_ptr<MapManager> Ptr;

 public:
  MapManager();
  ~MapManager();

  virtual bool Init() override;
  virtual bool Start() override;
  virtual void Stop() override;

 private:
  void LaserScanTransformTimer();

  void GenerateGridMapTimer();

 private:
  GridMap::Ptr local_grid_map_;  // 车身坐标系下的局部栅格地图
};

}  // namespace minco_local_planner::map_manager

#endif /* __MAP_MANAGER_H__ */
