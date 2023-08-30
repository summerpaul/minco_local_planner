/**
 * @Author: Yunkai Xia
 * @Date:   2023-07-20 09:31:18
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-08-24 17:38:21
 */
#include <stdint.h>

#ifndef __RIGID2D_H__
#define __RIGID2D_H__

#include "basis/data_type.h"
#include "math.h"

namespace minco_local_planner::utils {
using namespace basis;
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

// 带时间戳的控制
struct Twist2DStamped {
  Twist2DStamped() : twist(Twist2D()), timestamp(0) {}
  Twist2DStamped(const double &init_omega, const double &init_v_x,
                 const double &init_v_y, const double &time)
      : twist(Twist2D(init_omega, init_v_x, init_v_y)), timestamp(time) {}
  Twist2D twist;
  double timestamp;
};

class Transform2D {
 public:
  Transform2D() : trans_(Vec2f(0, 0)), radians_(0.0) {}

  explicit Transform2D(const Vec2f &trans) : trans_(trans), radians_(0.0) {}

  explicit Transform2D(const double radians)
      : trans_(Vec2f(0, 0)), radians_(radians) {}

  Transform2D(const Vec2f &trans, const double radians)
      : trans_(trans), radians_(radians) {}

  Transform2D(const double &x, const double &y, const double &radians)
      : trans_(Vec2f(x, y)), radians_(radians) {}

  Vec2f getTrans() const { return trans_; }

  double getRadians() const { return radians_; }

  void setTrans(const Vec2f &trans) { trans_ = trans; }

  void setRadius(const double &radius) { radians_ = radius; }

  void setZero() {
    trans_ = Vec2f(0, 0);
    radians_ = 0;
  }

  Vec2f operator()(const Vec2f &v) const {
    Vec2f result;
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

  Transform2D inv() const {
    double inv_radians = -radians_;
    Vec2f inv_trans;
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
    radians_ = normalizeAngleRad(radians_ + rhs.radians_);
    return *this;
  }

  void displacement(double &x, double &y, double &theta) {
    x = trans_.x();
    y = trans_.y();
    theta = radians_;
  }

 private:
  Vec2f trans_;
  double radians_;
};

inline Transform2D operator*(Transform2D lhs, const Transform2D &rhs) {
  lhs *= rhs;
  return lhs;
}

}  // namespace minco_local_planner::utils
#endif /* __RIGID2D_H__ */
