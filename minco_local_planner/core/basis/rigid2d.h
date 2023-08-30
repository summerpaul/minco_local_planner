/**
 * @Author: Xia Yunkai
 * @Date:   2023-08-24 22:47:51
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-08-30 22:13:36
 */
#include <stdint.h>

#ifndef __RIGID2D_H__
#define __RIGID2D_H__
#include "data_type.h"
#include "math.h"
namespace minco_local_planner::basis {

struct Twist2D {
 public:
  double omega;
  double v_x;
  double v_y;
  Twist2D() : omega(0), v_x(0), v_y(0) {}

  Twist2D(const double &init_omega, const double &init_v_x,
          const double &init_v_y)
      : omega(init_omega), v_x(init_v_x), v_y(init_v_y) {}
};

struct Pose2dStamped {
  Pose2dStamped() : pose(Vec3d::Zero()), timestamp(0.0) {}
  Pose2dStamped(const double &x, const double &y, const double &yaw,
                const double &time)
      : pose(Vec3d(x, y, yaw)), timestamp(time) {}
  Pose2d pose;
  double timestamp;
};

class Transform2D {
 public:
  Transform2D() : trans_(Vec2d(0, 0)), radians_(0.0) {}

  explicit Transform2D(const Vec2d &trans) : trans_(trans), radians_(0.0) {}

  explicit Transform2D(const double radians)
      : trans_(Vec2d(0, 0)), radians_(radians) {}

  Transform2D(const Vec2d &trans, const double radians)
      : trans_(trans), radians_(radians) {}

  Transform2D(const double &x, const double &y, const double &radians)
      : trans_(Vec2d(x, y)), radians_(radians) {}
  Transform2D(const Pose2d &pose) : trans_(pose.head(2)), radians_(pose[2]) {}

  const Vec2d GetTrans() const { return trans_; }

  const double GetRadians() const { return radians_; }

  void SetTrans(const Vec2d &trans) { trans_ = trans; }

  void SetRadius(const double &radius) { radians_ = radius; }

  void SetZero() {
    trans_ = Vec2d(0, 0);
    radians_ = 0;
  }

  Vec2d operator()(const Vec2d &v) const {
    Vec2d result;
    result.x() = v.x() * cos(radians_) - v.y() * sin(radians_);
    result.y() = v.x() * sin(radians_) + v.y() * cos(radians_);
    return result;
  }

  Twist2D operator()(const Twist2D &t) const {
    Twist2D result;
    result.omega = t.omega;
    result.v_x =
        trans_.y() * t.omega + cos(radians_) * t.v_x - sin(radians_) * t.v_y;
    result.v_y =
        -trans_.x() * t.omega + sin(radians_) * t.v_x + cos(radians_) * t.v_y;
    return result;
  }

  Transform2D Inv() const {
    double inv_radians = -radians_;
    Vec2d inv_trans;
    inv_trans.x() = -trans_.x() * cos(radians_) - trans_.y() * sin(radians_);
    inv_trans.y() = trans_.x() * sin(radians_) - trans_.y() * cos(radians_);
    Transform2D inv_result(inv_trans, inv_radians);
    return inv_result;
  }

  Transform2D &operator*=(const Transform2D &rhs) {
    trans_.x() = rhs.trans_.x() * cos(radians_) -
                 rhs.trans_.y() * sin(radians_) + trans_.x();
    trans_.y() = rhs.trans_.x() * sin(radians_) +
                 rhs.trans_.y() * cos(radians_) + trans_.y();
    radians_ = NormalizeAngleRad(radians_ + rhs.radians_);
    return *this;
  }

  void Displacement(double &x, double &y, double &theta) {
    x = trans_.x();
    y = trans_.y();
    theta = radians_;
  }

 private:
  Vec2d trans_;
  double radians_;
};

inline Vec2d SubVec2d(const Pose2d &origin, const Vec2d &pose) {
  Vec2d ret;
  const auto cos_ = std::cos(origin[2]);
  const auto sin_ = std::sin(origin[2]);
  ret[0] = (pose[0] - origin[0]) * cos_ + (pose[1] - origin[1]) * sin_;
  ret[1] = -(pose[0] - origin[0]) * sin_ + (pose[1] - origin[1]) * cos_;

  return ret;
}

inline Vec2d AddVec2d(const Pose2d &origin, const Vec2d &incr) {
  Vec2d ret;
  const auto cos_ = std::cos(origin[2]);
  const auto sin_ = std::sin(origin[2]);
  ret[0] = origin[0] + incr[0] * cos_ - incr[1] * sin_;
  ret[1] = origin[1] + incr[0] * sin_ + incr[1] * cos_;
  return ret;
}

inline Pose2d AddPose2d(const Pose2d &origin, const Pose2d &incr) {
  Pose2d ret;
  ret[0] = origin[0] + incr[0] * cos(origin[2]) - incr[1] * sin(origin[2]);
  ret[1] = origin[1] + incr[0] * sin(origin[2]) + incr[1] * cos(origin[2]);
  ret[2] = NormalizeAngleRad(origin[2] + incr[2]);
  return ret;
}

inline Pose2d SubPose2d(const Pose2d &origin, const Pose2d &pose) {
  Pose2d ret;
  const auto cos_ = std::cos(origin[2]);
  const auto sin_ = std::sin(origin[2]);
  ret[0] = (pose[0] - origin[0]) * cos_ + (pose[1] - origin[1]) * sin_;
  ret[1] = -(pose[0] - origin[0]) * sin_ + (pose[1] - origin[1]) * cos_;
  ret[2] = NormalizeAngleRad(pose[2] - origin[2]);
  return ret;
}

inline Eigen::Quaterniond GetQuaterion(const Pose2d &p) {
  double sy = std::sin(p[2] * 0.5);
  double cy = std::cos(p[2] * 0.5);
  double sp = 0.;
  double cp = 1.;
  double sr = 0.;
  double cr = 1.;
  double w = cr * cp * cy + sr * sp * sy;
  double x = sr * cp * cy - cr * sp * sy;
  double y = cr * sp * cy + sr * cp * sy;
  double z = cr * cp * sy - sr * sp * cy;
  return Eigen::Quaterniond(w, x, y, z);
}

}  // namespace minco_local_planner::basis
#endif /* __RIGID2D_H__ */
