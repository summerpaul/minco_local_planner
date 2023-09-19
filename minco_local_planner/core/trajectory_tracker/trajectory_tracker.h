/**
 * @Author: Xia Yunkai
 * @Date:   2023-09-05 00:07:46
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-09-05 00:13:12
 */
#include <stdint.h>

#ifndef __TRAJECTORY_TRACKER_H__
#define __TRAJECTORY_TRACKER_H__
#include "basis/base_module.h"
#include "basis/rigid2d.h"
#include "basis/trajectory.h"
#include "basis/vehicle_pose.h"
#include "basis/logger.h"
namespace minco_local_planner::trajectory_optimizer {

using namespace basis;

enum TrackingStatus { NEW_PATH, NO_PATH, FAR_FROM_PATH, FOLLOWING, GOAL };

struct TrackingResult {
  TrackingStatus tracking_state{NO_PATH};
  double distance_remains{0};         // 轨迹跟踪的剩余距离
  double lateral_error{0};            // 轨迹跟踪过程中的横向误差
  double heading_error{0};            // 轨迹跟踪过程中的角度误差
  Twist2D twist;                      // 规划的控制输出
  Vec2d neaset_pt{Vec2f::Zero()};     //
  Vec2d look_head_pt{Vec2f::Zero()};  // 轨迹跟踪的前视点
  VehiclePose cur_state;
  double turning_curve{0};  // 车辆形式过程中的转弯半径
  double target_speed{0};
};
class TrajectoryTracker : public BaseModule {
 public:
  virtual bool Init() override;
  virtual bool Start() override;
  virtual void Stop() override;
};

}  // namespace minco_local_planner::trajectory_optimizer

#endif /* __TRAJECTORY_TRACKER_H__ */
