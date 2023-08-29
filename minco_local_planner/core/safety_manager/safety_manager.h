/**
 * @Author: Xia Yunkai
 * @Date:   2023-08-24 20:05:56
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-08-29 20:02:33
 */
#include <stdint.h>

#ifndef __SAFETY_MANAGER_H__
#define __SAFETY_MANAGER_H__

#include <map>

#include "basis/base_module.h"
#include "basis/logger.h"
#include "basis/trajectory.h"
#include "bounding_box.h"
#include "config_manager/config_data.h"
namespace minco_local_planner::safety_manager {
using namespace config_manager;
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

 private:
  void GenerateBoundingBoxes();

  void CheckBoundingBoxes();

  void ChangeVehicleSafetyState(const SafetyStatus& new_status);
  void ChangePathSafetyState(const SafetyStatus& new_status);

 private:
  SafetyManagerConfig cfg_;
  SafetyStatus vehicle_safety_status_;
  SafetyStatus path_safety_status_;
  std::map<int, BoundingBox> bouding_boxes_;
  Path2d safe_check_path_;

  const std::string safety_status_str_[6] = {"SAFE",   "SLOW_DOWN",
                                             "CREEP",  "EMERGENCY_STOP",
                                             "REPLAN", "OVER_MAX_KAPPA"};
};
}  // namespace minco_local_planner::safety_manager

#endif /* __SAFETY_MANAGER_H__ */
