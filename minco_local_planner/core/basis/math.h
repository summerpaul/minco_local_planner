/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-24 14:49:26
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-08-30 15:37:21
 */
#include <stdint.h>

#ifndef __MATH_H__
#define __MATH_H__
#include <cmath>
#include <vector>
namespace minco_local_planner::basis {

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
T NormalizeAngleDeg(T deg) {
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
T NormalizeAngleRad(T rad) {
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

inline int Sign(const double &num) { return num == 0 ? 0 : num < 0 ? -1 : 1; }

inline double Mod(double value, double modulus) {
  return fmod(fmod(value, modulus) + modulus, modulus);
}

inline double Intbound(double s, double ds) {
  // Find the smallest positive t such that s+t*ds is an integer.
  if (ds < 0) {
    return Intbound(-s, -ds);
  } else {
    s = Mod(s, 1);
    // problem is now s+t*ds = 1
    return (1 - s) / ds;
  }
}

}  // namespace minco_local_planner::basis
#endif /* __MATH_H__ */
