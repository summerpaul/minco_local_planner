/**
 * @Author: Yunkai Xia
 * @Date:   2023-04-19 15:36:30
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-06-09 20:10:42
 */
#include <stdint.h>

#ifndef __QUINTIC_POLYNOMIAL_H__
#define __QUINTIC_POLYNOMIAL_H__

#include "basis/data_type.h"
namespace parametric_curve {
using namespace basis;
class QuinticPolynomial {
 public:
  // current parameter at t=0
  decimal_t xs;
  decimal_t vxs;
  decimal_t axs;

  // parameters at target t=t_j
  decimal_t xe;
  decimal_t vxe;
  decimal_t axe;

  // function parameters
  decimal_t a0, a1, a2, a3, a4, a5;

  QuinticPolynomial() {}
  QuinticPolynomial(const decimal_t &xs_, const decimal_t &vxs_,
                    const decimal_t &axs_, const decimal_t &xe_,
                    const decimal_t &vxe_, const decimal_t &axe_,
                    const decimal_t &T)
      : xs(xs_),
        vxs(vxs_),
        axs(axs_),
        xe(xe_),
        vxe(vxe_),
        axe(axe_),
        a0(xs_),
        a1(vxs_),
        a2(axs_ / 2.0) {
    Eigen::Matrix3f A;
    A << std::pow(T, 3), std::pow(T, 4), std::pow(T, 5), 3 * std::pow(T, 2),
        4 * std::pow(T, 3), 5 * std::pow(T, 4), 6 * T, 12 * std::pow(T, 2),
        20 * std::pow(T, 3);
    // std::cout << "A is " << A << std::endl;
    Eigen::Vector3f B;
    B << xe - a0 - a1 * T - a2 * std::pow(T, 2), vxe - a1 - 2 * a2 * T,
        axe - 2 * a2;

    Eigen::Vector3f c_eigen = A.colPivHouseholderQr().solve(B);
    a3 = c_eigen[0];
    a4 = c_eigen[1];
    a5 = c_eigen[2];
  }

  decimal_t calcPoint(const decimal_t &t) {
    return a0 + a1 * t + a2 * std::pow(t, 2) + a3 * std::pow(t, 3) +
           a4 * pow(t, 4) + a5 * pow(t, 5);
  }
  decimal_t calcFirstDerivative(const decimal_t &t) {
    return a1 + 2 * a2 * t + 3 * a3 * std::pow(t, 2) + 4 * a4 * pow(t, 3) +
           5 * a5 * pow(t, 4);
  }

  decimal_t calcSecondDerivative(const decimal_t &t) {
    return 2 * a2 + 6 * a3 * t + 12 * a4 * pow(t, 2) + 20 * a5 * pow(t, 3);
  }

  decimal_t calcThirdDerivative(const decimal_t &t) {
    return 6 * a3 + 24 * a4 * t + 60 * a5 * pow(t, 2);
  }
};
}  // namespace parametric_curve

#endif /* __QUINTIC_POLYNOMIAL_H__ */
