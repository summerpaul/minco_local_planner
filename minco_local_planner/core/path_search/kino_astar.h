/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-30 14:18:30
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-09-04 16:04:29
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
  double EstimateHeuristic(const VehiclePose& currt_pt,
                           const VehiclePose& target_pt, double& optimal_time);
  Vec_d Quartic(const double& a, const double& b, const double& c,
                const double& d, const double& e);
  Vec_d Cubic(const double& a, const double& b, const double& c,
              const double& d);

  bool ComputeShotTraj(const VehiclePose& state1, const VehiclePose& state2,
                       const double& time_to_goal);
  void StateTransit(VehiclePose& state0, VehiclePose& state1, const Vec2d& um,
                    const double& tau);

 private:
  KinoAstarConfig::Ptr cfg_;
  Eigen::MatrixXd coef_shot_, coef_shot_vel_, coef_shot_acc_;
  double t_shot_{0};
  bool is_shot_succ_{false};
  Vec2d end_vel_;
};
}  // namespace minco_local_planner::path_search

#endif /* __KINO_ASTAR_H__ */
