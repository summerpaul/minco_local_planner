/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-24 14:49:26
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-08-24 15:02:16
 */
#include <stdint.h>

#ifndef __MATH_H__
#define __MATH_H__
#include <cmath>
#include <vector>
namespace minco_local_planner::utils {

#define EPSION 1E-4

template <typename T>
constexpr T DegToRad(const T &deg) {
  return M_PI * deg / 180.;
}
template <typename T>
constexpr T RadToDeg(const T &rad) {
  return 180. * rad / M_PI;
}

template <typename T>
T NormalizeAngleDeg(const T &deg) {
  const T degPi = T(180.0);
  while (deg > degPi) {
    deg -= 2. * degPi;
  }
  while (deg < -degPi) {
    deg += 2. * degPi;
  }
  return deg;
}

template <typename T>
T NormalizeAngleRad(const T &rad) {
  const T radPi = T(M_PI);
  while (rad > radPi) {
    rad -= 2. * radPi;
  }
  while (rad < -radPi) {
    rad += 2. * radPi;
  }
  return rad;
}

template <typename T>
inline int Sign(const T &num) {
  if (num < 0)
    return -1;
  else if (num > 0)
    return 1;
  else
    return 0;
}

template <typename T>
inline bool IsClose(const T &a, const T &b) {
  return std::fabs(a - b) < EPSION;
}

template <typename T>
inline T Norm(const T &x, const T &y) {
  return sqrt(pow(x, 2) + pow(y, 2));
}

template <typename T>
inline bool IsLarge(const T &value_1, const T &value_2) {
  if (value_1 > value_2 && std::fabs(value_1 - value_2) > EPSION) {
    return true;
  }
  return false;
}

template <typename T>
inline bool IsSmall(const T &value_1, const T &value_2) {
  if (value_1 < value_2 && std::fabs(value_1 - value_2) > EPSION) {
    return true;
  }
  return false;
}

template <typename T>
inline bool IsEqual(const T &value_1, const T &value_2) {
  if (std::fabs(value_1 - value_2) <= EPSION) {
    return true;
  } 
  return false;
}

}  // namespace minco_local_planner::utils
#endif /* __MATH_H__ */
