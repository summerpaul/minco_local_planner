/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-24 17:23:47
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-08-27 21:25:40
 */
#include <stdint.h>

#ifndef __RUNTIME_MANAGER_H__
#define __RUNTIME_MANAGER_H__

#include "basis/base_module.h"
#include "basis/data_type.h"
#include "basis/laser_scan.h"
#include "basis/logger.h"
#include "basis/rigid2d.h"
#include "basis/vehicle_pose.h"
#include "config_manager/config_data.h"
namespace minco_local_planner::runtime_manager {

enum class RuntimeStatus { NORMAL, MISS_POSE, MISS_SCAN, MISS_TWIST, MISS_ALL };

using namespace basis;
class RuntimeManager : public BaseModule {
 public:
  typedef std::shared_ptr<RuntimeManager> Ptr;

 public:
  RuntimeManager();
  ~RuntimeManager();

  virtual bool Init() override;
  virtual bool Start() override;
  virtual void Stop() override;
  //   只有定位数据
  void UpdatePose2d(const Pose2d& pose);
  //   激光雷达
  void UpdateScan(const LaserScan& scan);
  //   底盘速度
  void UpdateChassisTwist(const Twist2D& twist);
  // 更新定位+速度
  void UpdateVehiclePose(const VehiclePose& pose);

  void UpdateTwist(const Twist2D& twist);

  const RuntimeStatus& GetStatus() const { return status_; }
  const VehiclePose& GetVehiclePose() const { return cur_pos_; }
  const LaserScan& GetLaserScan() const { return cur_scan_; }
  const Twist2D& GetTwist() const { return cur_twist_; }

 private:
  void CheckRuntimeTimer();

 private:
  std::mutex pose_mtx_, scan_mtx_, twist_mtx_;
  double pose_t_, scan_t_, twist_t_;
  LaserScan cur_scan_;
  VehiclePose cur_pos_;
  Twist2D cur_twist_;
  RuntimeStatus status_;
  config_manager::RuntimeMangerConfig cfg_;
};
}  // namespace minco_local_planner::runtime_manager

#endif /* __RUNTIME_MANAGER_H__ */
