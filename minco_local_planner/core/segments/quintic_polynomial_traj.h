/**
 * @Author: Yunkai Xia
 * @Date:   2023-04-20 15:42:18
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-06-09 20:10:33
 */
#include <stdint.h>

#ifndef __QUINTIC_POLYNOMIAL_TRAJ_H__
#define __QUINTIC_POLYNOMIAL_TRAJ_H__
#include "basis/trajectory.h"
#include "quintic_polynomial.h"
namespace parametric_curve {
using namespace basis;

class QuinticPolynomialTraj {
 public:
  QuinticPolynomialTraj() {}
  QuinticPolynomialTraj(const Vec2f &start_pose, const Vec2f &end_pose,
                        const Vec2f &start_vel, const Vec2f &end_vel,
                        const Vec2f &start_acc, const Vec2f &end_acc,
                        const decimal_t &time) {
    x_ = QuinticPolynomial(start_pose.x(), start_vel.x(), start_acc.x(),
                           end_pose.x(), end_vel.x(), end_acc.x(), time);
    y_ = QuinticPolynomial(start_pose.y(), start_vel.y(), start_acc.y(),
                           end_pose.y(), end_vel.y(), end_acc.y(), time);
    time_ = time;
  }
  Vec2f clacPoint(const decimal_t &t) {
    return Vec2f(x_.calcPoint(t), y_.calcPoint(t));
  }
  Vec2f clacVel(const decimal_t &t) {
    return Vec2f(x_.calcFirstDerivative(t), y_.calcFirstDerivative(t));
  }
  Vec2f calcAcc(const decimal_t &t) {
    return Vec2f(x_.calcSecondDerivative(t), y_.calcSecondDerivative(t));
  }

  TrajectoryPoint calcTrajPoint(const decimal_t &t) {
    Vec2f pt = clacPoint(t);
    Vec2f vel = clacVel(t);
    Vec2f acc = calcAcc(t);
    TrajectoryPoint traj_pt;
    const decimal_t theta =
        normalizeAngleRad(atan2(vel.y(), vel.x()));  // theta
    const decimal_t kappa =
        (vel.x() * acc.y() - vel.y() * acc.x()) /
        pow((vel.x() * vel.x() + vel.y() * vel.y()), 1.5);  // kappa
    traj_pt.setPos(pt);
    traj_pt.setKappa(kappa);
    traj_pt.setTheta(theta);
    traj_pt.setVel(vel.norm());
    return traj_pt;
  }

  Trajectory getTraj(const decimal_t &dt = 0.01) {
    Trajectory traj;
    for (decimal_t t = 0.0; t < time_; t += dt) {
      auto traj_pt = calcTrajPoint(t);
      traj.emplace_back_pt(traj_pt);
    }
    return traj;
  }

  Path getPath(const decimal_t &interval = 0.1) {
    Path path;
    for (decimal_t t = 0.0; t < time_; t += 0.01) {
      auto pt = clacPoint(t);
      if (path.size() == 0.0) {
        path.emplace_back(pt);
      } else {
        Vec2f back_pt = path.back();
        Vec2f diff = back_pt - pt;
        if (diff.norm() >= interval) {
          path.emplace_back(pt);
        }
      }
    }
    return path;
  }

 private:
  QuinticPolynomial x_;
  QuinticPolynomial y_;
  decimal_t time_;
};

}  // namespace parametric_curve

#endif /* __QUINTIC_POLYNOMIAL_TRAJ_H__ */
