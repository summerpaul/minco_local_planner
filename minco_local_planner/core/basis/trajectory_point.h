/**
 * @Author: Xia Yunkai
 * @Date:   2023-08-24 20:54:13
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-08-24 23:10:44
 */
#include <stdint.h>

#ifndef __TRAJECTORY_POINT_H__
#define __TRAJECTORY_POINT_H__

#include "data_type.h"
namespace minco_local_planner::basis {

struct TrajectoryPoint : public Vec6d {
  const double getX() const { return (*this)[0]; }
  const double getY() const { return (*this)[1]; }
  const double getYaw() const { return (*this)[2]; }
  const double getKappa() const { return (*this)[3]; }
  const double getS() const { return (*this)[4]; }
  const double getVel() const { return (*this)[5]; }
  const Vec2d getPos() const { return (*this).head(2); }
  const Pose2d GetPose2d() const { return (*this).head(3); }

  void SetX(const double &x) { (*this)[0] = x; }
  void SetY(const double &y) { (*this)[1] = y; }
  void SetTheta(const double &theta) { (*this)[2] = theta; }
  void SetKappa(const double &kappa) { (*this)[3] = kappa; }
  void SetS(const double &s) { (*this)[4] = s; }
  void SetVel(const double &vel) { (*this)[5] = vel; }
  void SetPos(const Vec2d &pos) { (*this).head(2) = pos; }
  void SetPose2d(const Pose2d &pose) { (*this).head(3) = pose; }
};
}  // namespace minco_local_planner::basis

#endif /* __TRAJECTORY_POINT_H__ */
