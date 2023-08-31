/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-31 14:32:41
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-08-31 17:29:22
 */
#include <stdint.h>

#ifndef __ASTAR_H__
#define __ASTAR_H__
#include "path_search.h"
namespace minco_local_planner::path_search {
class Astar : public PathSearch {
 public:
  Astar();
  virtual int Search(const VehiclePose& start_pos, const VehiclePose& end_pos,
                     const Vec2d& init_ctrl) override;

  virtual void Reset() override;

  virtual bool Init() override;
  virtual bool Start() override;
  virtual void Stop() override;

 protected:
  bool CheckVehiclePose(const VehiclePose& pose);

 private:
  vec_Vec2i motions_;
};
}  // namespace minco_local_planner::path_search

#endif /* __ASTAR_H__ */
