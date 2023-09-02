/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-30 14:18:30
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-09-03 04:20:46
 */
#include <stdint.h>

#ifndef __KINO_ASTAR_H__
#define __KINO_ASTAR_H__
#include "path_search.h"
namespace minco_local_planner::path_search {
using namespace path_search;
using namespace basis;
class KinoAstar : public PathSearch {
 public:
  KinoAstar();

  virtual int Search(const VehiclePose& start_pos, const VehiclePose& end_pos,
                     const Vec2d& init_ctrl) override;

  virtual void Reset() override;
  virtual bool Init() override;
  virtual bool Start() override;
  virtual void Stop() override;
  virtual void GetPath2D(Path2d& path) override;

 private:
  KinoAstarConfig::Ptr cfg_;
};
}  // namespace minco_local_planner::path_search

#endif /* __KINO_ASTAR_H__ */
