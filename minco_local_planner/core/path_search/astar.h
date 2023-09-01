/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-31 14:32:41
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-09-01 18:50:16
 */
#include <stdint.h>

#ifndef __ASTAR_H__
#define __ASTAR_H__
#include "path_search.h"

namespace minco_local_planner::path_search {
class Astar : public PathSearch {
 public:
  Astar();
  ~Astar();
  virtual int Search(const VehiclePose& start_pos, const VehiclePose& end_pos,
                     const Vec2d& init_ctrl) override;

  virtual void Reset() override;

  virtual bool Init() override;
  virtual bool Start() override;
  virtual void Stop() override;
  virtual void GetPath2D(Path2d& path) override;

 protected:
  // bool CheckVehiclePose(const VehiclePose& pose);
 private:
  double GetEuclHeu(const Vec2d& x1, const Vec2d& x2,
                    const double& tie_breaker = 1.01);
  double GetDiagHeu(const Vec2d& x1, const Vec2d& x2,
                    const double& tie_breaker = 1.01);
  double GetManhHeu(const Vec2d& x1, const Vec2d& x2,
                    const double& tie_breaker = 1.01);

  void RetrievePath(PathNodePtr end_node);

 private:
  vec_Vec2i motions_;
  Vec2d end_pt_;
  AstarConfig cfg_;
};
}  // namespace minco_local_planner::path_search

#endif /* __ASTAR_H__ */
