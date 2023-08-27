/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-25 10:29:08
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-08-25 11:19:22
 */
#include <stdint.h>

#ifndef __PLANNER_CONST_H__
#define __PLANNER_CONST_H__

namespace minco_local_planner::basis {



enum class TrackingStatus { NEW_PATH, NO_PATH, FAR_FROM_PATH, FOLLOWING, GOAL };

enum class PlanStatus {
  INIT,
  GLOBAL_PLAN,
  TRAJECTORY_OPT,
  ROTATION_CAL,
  ROTATION,
  TRAJECTORY_TRACKING,
  FAR_FROM_PATH,
  STOP,
  REACH_SEGMENT_END,
  REACH_GOAL,
  OPT_ERROR,
  PATH_ERROR
};

enum class SafetyStatus {
  SAFE,
  SLOW_DOWN,
  CREEP,
  EMERGENCY_STOP,
  REPLAN,
  OVER_MAX_KAPPA
};

}  // namespace minco_local_planner::basis

#endif /* __PLANNER_CONST_H__ */
