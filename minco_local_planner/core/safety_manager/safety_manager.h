/**
 * @Author: Xia Yunkai
 * @Date:   2023-08-24 20:05:56
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-08-30 15:44:11
 */
#include <stdint.h>

#ifndef __SAFETY_MANAGER_H__
#define __SAFETY_MANAGER_H__

#include <map>
#include <mutex>

#include "basis/base_module.h"
#include "basis/logger.h"
#include "basis/trajectory.h"
#include "bounding_box.h"
#include "config_manager/config_data.h"
#include "raycast.h"

namespace minco_local_planner::safety_manager {
using namespace config_manager;
using namespace basis;
enum class SafetyStatus {
  SAFE,
  SLOW_DOWN,
  CREEP,
  STOP,
  REPLAN,
  OVER_MAX_KAPPA
};

enum BoundingBoxType { VEHICLE = 0, STOP, CREEP, SLOW_DOWN };

class SafetyManager : public BaseModule {
 public:
  typedef std::shared_ptr<SafetyManager> Ptr;

 public:
  SafetyManager();
  ~SafetyManager();
  virtual bool Init() override;
  virtual bool Start() override;
  virtual void Stop() override;

  const SafetyStatus GetVehicleSafetyStatus() const {
    return vehicle_safety_status_;
  }

  const SafetyStatus GetPathSafetyStatus() const { return path_safety_status_; }
  const Path2d& GetSafeCheckPath() const { return safe_check_path_; }
  const std::map<int, BoundingBox>& GetBoundingBoxs() const {
    return bouding_boxes_;
  }

  void UpdateCheckTraj(const Trajectory& traj);

  void SetEndPoit(const Vec2d& end_pt) { end_pt_ = end_pt; }

 
  bool CheckPose2dObs(const Pose2d& check_pt);

 private:
  void GenerateBoundingBoxes();

  void CheckTrajectoryTimer();
  void CheckBoundingBoxesTimer();

  void ChangeVehicleSafetyState(const SafetyStatus& new_status);
  void ChangePathSafetyState(const SafetyStatus& new_status);

 private:
  SafetyManagerConfig cfg_;
  SafetyStatus vehicle_safety_status_;
  SafetyStatus path_safety_status_;
  std::map<int, BoundingBox> bouding_boxes_;  // 方便可视化
  BoundingBox vehicle_box_, stop_box_, creep_box_, slow_down_box_;
  Points2d vehicle_box_points_;
  Path2d safe_check_path_;
  Vec2d end_pt_;
  std::mutex safe_check_path_mtx_;
  std::mutex check_traj_mtx_;
  Trajectory check_traj_;
  double check_traj_length_;

  const std::string safety_status_str_[6] = {"SAFE",   "SLOW_DOWN",
                                             "CREEP",  "EMERGENCY_STOP",
                                             "REPLAN", "OVER_MAX_KAPPA"};
};
}  // namespace minco_local_planner::safety_manager

#endif /* __SAFETY_MANAGER_H__ */
