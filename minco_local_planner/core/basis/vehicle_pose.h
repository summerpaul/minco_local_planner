/**
 * @Author: Xia Yunkai
 * @Date:   2023-08-24 20:53:21
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-08-30 10:42:40
 */
#include <stdint.h>

#ifndef __VEHICLE_POSE_H__
#define __VEHICLE_POSE_H__

#include "data_type.h"
namespace minco_local_planner::basis {
struct VehiclePose : public Vecd<9> {
  VehiclePose() { this->setZero(); }
  void SetZero() { this->setZero(); }
  const double GetX() const { return (*this)[0]; }
  const double GetY() const { return (*this)[1]; }
  const double GetYaw() const { return (*this)[2]; }
  const double GetVelX() const { return (*this)[3]; }
  const double GetVelY() const { return (*this)[4]; }
  const double GetAngularVel() const { return (*this)[5]; }
  const double GetAccX() const { return (*this)[6]; }
  const double GetAccY() const { return (*this)[7]; }
  const double GetAngularAcc() const { return (*this)[8]; }

  const Vec2d GetPos() const { return (*this).head(2); }
  const Vec2d GetVel() const { return (*this).segment(3, 2); }
  const Vec2d GetAcc() const { return (*this).segment(6, 2); }
  const Pose2d GetPose2d() const { return (*this).head(3); }

  void SetX(const double &x) { (*this)[0] = x; }
  void SetY(const double &y) { (*this)[1] = y; }
  void SetYaw(const double &yaw) { (*this)[2] = yaw; }
  void SetVelX(const double &line_vel_x) { (*this)[3] = line_vel_x; }
  void SetVelY(const double &line_vel_y) { (*this)[4] = line_vel_y; }
  void SetAngularVel(const double &angular_vel) { (*this)[5] = angular_vel; }
  void SetAccX(const double &line_acc_x) { (*this)[6] = line_acc_x; }
  void SetAccY(const double &line_acc_y) { (*this)[7] = line_acc_y; }
  void SetAngularAcc(const double &angular_acc) { (*this)[8] = angular_acc; }

  void SetPos(const Vec2d &pos) { (*this).head(2) = pos; }
  void SetVel(const Vec2d &vel) { (*this).segment(3, 2) = vel; }
  void SetAcc(const Vec2d &acc) { (*this).segment(6, 2) = acc; }
  void SetPose2d(const Pose2d &pose) { (*this).head(3) = pose; }
};
}  // namespace minco_local_planner::basis

#endif /* __VEHICLE_POSE_H__ */
