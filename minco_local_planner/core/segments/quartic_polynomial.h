/**
 * @Author: Yunkai Xia
 * @Date:   2023-04-19 15:40:34
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-06-09 20:10:29
 */
#include <stdint.h>

#ifndef __QUARTIC_POLYNOMIAL_H__
#define __QUARTIC_POLYNOMIAL_H__

#include "basis/data_type.h"
namespace parametric_curve {
using namespace basis;
class QuarticPolynomial {
 public:
  // current parameter at t=0
  decimal_t xs;
  decimal_t vxs;
  decimal_t axs;

  // parameters at target t=t_j
  decimal_t vxe;
  decimal_t axe;

  // function parameters
  decimal_t a0, a1, a2, a3, a4;

  QuarticPolynomial(){};

  QuarticPolynomial(const decimal_t &xs_, const decimal_t &vxs_,
                    const decimal_t &axs_, const decimal_t &vxe_,
                    const decimal_t &axe_, const decimal_t &T)
      : xs(xs_),
        vxs(vxs_),
        axs(axs_),
        vxe(vxe_),
        axe(axe_),
        a0(xs_),
        a1(vxs_),
        a2(axs_ / 2.0) {
    Eigen::Matrix2f A;
    A << 3 * std::pow(T, 2), 4 * std::pow(T, 3), 6 * T, 12 * std::pow(T, 2);
    Eigen::Vector2f B;
    B << vxe - a1 - 2 * a2 * T, axe - 2 * a2;

    Eigen::Vector2f c_eigen = A.colPivHouseholderQr().solve(B);
    a3 = c_eigen[0];
    a4 = c_eigen[1];
  };

  decimal_t calcPoint(const decimal_t &t) {
    return a0 + a1 * t + a2 * std::pow(t, 2) + a3 * std::pow(t, 3) +
           a4 * std::pow(t, 4);
  };

  decimal_t calcFirstDerivative(const decimal_t &t) {
    return a1 + 2 * a2 * t + 3 * a3 * std::pow(t, 2) + 4 * a4 * std::pow(t, 3);
  };

  decimal_t calcSecondDerivative(const decimal_t &t) {
    return 2 * a2 + 6 * a3 * t + 12 * a4 * std::pow(t, 2);
  };

  decimal_t calcThirdDerivative(const decimal_t &t) {
    return 6 * a3 + 24 * a4 * t;
  };
};
}  // namespace parametric_curve

#endif /* __QUARTIC_POLYNOMIAL_H__ */
