/**
 * @Author: Xia Yunkai
 * @Date:   2023-08-29 20:48:46
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-08-29 21:02:37
 */
#include <stdint.h>

#ifndef __TRAJECTORY_OPTIMIZER_H__
#define __TRAJECTORY_OPTIMIZER_H__

#include "basis/base_module.h"
#include "basis/trajectory.h"
namespace minco_local_planner::trajectory_optimizer {
using namespace basis;

class TrajectoryOptimizer : public BaseModule {
 public:
  virtual bool Init() override;
  virtual bool Start() override;
  virtual void Stop() override;

  virtual bool optimize() = 0;

  void setRefTrajectory(const Trajectory& ref_traj) { ref_traj_ = ref_traj; }

  const Trajectory& getOptTrajectory() const { return opt_traj_; }

 protected:
  Trajectory opt_traj_;
  Trajectory ref_traj_;
};

}  // namespace minco_local_planner::trajectory_optimizer

#endif /* __TRAJECTORY_OPTIMIZER_H__ */
