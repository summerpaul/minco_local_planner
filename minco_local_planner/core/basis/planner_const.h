/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-25 10:29:08
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-08-31 10:36:23
 */
#include <stdint.h>

#ifndef __PLANNER_CONST_H__
#define __PLANNER_CONST_H__

namespace minco_local_planner::basis {

enum PathSearchType { ASTAR = 0, LAZY_THETA_ASTAR, KINO_ASTAR };
enum TrajectoryOptimizerType { BSPLINE = 0, MINCO };
enum TrajectoryTrackerType { PURE_PURSUIT = 0, LQR, MPC };

}  // namespace minco_local_planner::basis

#endif /* __PLANNER_CONST_H__ */
