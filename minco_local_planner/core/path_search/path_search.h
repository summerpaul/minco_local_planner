/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-30 13:40:34
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-09-03 04:19:25
 */
#include <stdint.h>

#ifndef __PATH_SEARCH_H__
#define __PATH_SEARCH_H__
// #include <boost/functional/hash.hpp>
#include <memory>

#include "basis/vehicle_pose.h"
#include "config_manager/config_data.h"
#include "map_manager/grid_map.h"
#include "type.h"

namespace minco_local_planner::path_search {
using namespace basis;
using namespace map_manager;
using namespace config_manager;

class PathSearch : public BaseModule {
 public:
  typedef std::unique_ptr<PathSearch> Ptr;

  PathSearch(const std::string& name) : BaseModule(name) {}
  //   设置规划的地图
  void SetMap(const GridMap::Ptr& map) { map_ptr_ = map; }
  // carlike 混合A星使用
  virtual int Search(const VehiclePose& start_pos, const VehiclePose& end_pos,
                     const Vec2d& init_ctrl) = 0;

  // 重置数据
  virtual void Reset() = 0;

  virtual void GetPath2D(Path2d& path) = 0;

 protected:
  // virtual bool CheckVehiclePose(const VehiclePose& pose) = 0;

 protected:
  GridMap::Ptr map_ptr_;
  std::vector<PathNodePtr> path_node_pool_;
  int use_node_num_, iter_num_;

  std::vector<PathNodePtr> path_nodes_;
  NodeHashTable<PathNodePtr> expanded_nodes_;
  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator>
      open_set_;
};

}  // namespace minco_local_planner::path_search
#endif /* __PATH_SEARCH_H__ */
