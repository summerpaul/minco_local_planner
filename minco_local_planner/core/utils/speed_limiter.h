/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-24 15:04:24
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-08-24 15:09:02
 */
#include <stdint.h>

#ifndef __SPEED_LIMITER_H__
#define __SPEED_LIMITER_H__
#include <algorithm>
#include <iostream>
namespace minco_local_planner::utils {

template <typename T>
T clamp(T x, T min, T max) {
  return std::min(std::max(min, x), max);
}

class SpeedLimiter {
 public:
  SpeedLimiter(const bool& has_velocity_limits = false,
               const bool& has_acceleration_limits = false,
               const bool& has_jerk_limits = false,
               const double& min_velocity = 0.0,
               const double& max_velocity = 0.0,
               const double& min_acceleration = 0.0,
               const double& max_acceleration = 0.0,
               const double& min_jerk = 0.0, const double& max_jerk = 0.0)
      : has_velocity_limits(has_velocity_limits),
        has_acceleration_limits(has_acceleration_limits),
        has_jerk_limits(has_jerk_limits),
        min_velocity(min_velocity),
        max_velocity(max_velocity),
        min_acceleration(min_acceleration),
        max_acceleration(max_acceleration),
        min_jerk(min_jerk),
        max_jerk(max_jerk) {}

  double Limit(double& v, const double& v0, const double& v1,
               const double& dt) {
    const double tmp = v;

    LimitJerk(v, v0, v1, dt);
    LimitAcceleration(v, v0, dt);
    LimitVelocity(v);

    return tmp != 0.0 ? v / tmp : 1.0;
  }

  double LimitVelocity(double& v) {
    const double tmp = v;

    if (has_velocity_limits) {
      v = clamp(v, min_velocity, max_velocity);
    }

    return tmp != 0.0 ? v / tmp : 1.0;
  }

  double LimitAcceleration(double& v, const double& v0, const double& dt) {
    const double tmp = v;

    if (has_acceleration_limits) {
      const double dv_min = min_acceleration * dt;
      const double dv_max = max_acceleration * dt;

      const double dv = clamp(v - v0, dv_min, dv_max);

      v = v0 + dv;
    }

    return tmp != 0.0 ? v / tmp : 1.0;
  }

  double LimitJerk(double& v, const double& v0, const double& v1,
                   const double& dt) {
    const double tmp = v;

    if (has_jerk_limits) {
      const double dv = v - v0;
      const double dv0 = v0 - v1;

      const double dt2 = 2. * dt * dt;

      const double da_min = min_jerk * dt2;
      const double da_max = max_jerk * dt2;

      const double da = clamp(dv - dv0, da_min, da_max);

      v = v0 + dv0 + da;
    }

    return tmp != 0.0 ? v / tmp : 1.0;
  }

 public:
  // Enable/Disable velocity/acceleration/jerk limits:
  bool has_velocity_limits;
  bool has_acceleration_limits;
  bool has_jerk_limits;

  // Velocity limits:
  double min_velocity;
  double max_velocity;

  // Acceleration limits:
  double min_acceleration;
  double max_acceleration;

  // Jerk limits:
  double min_jerk;
  double max_jerk;
};
}  // namespace minco_local_planner::utils

#endif /* __SPEED_LIMITER_H__ */
