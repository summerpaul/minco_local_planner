/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-24 15:09:43
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-08-24 15:10:37
 */
#include <stdint.h>

#ifndef __PID_CONTROL_H__
#define __PID_CONTROL_H__

namespace minco_local_planner::utils {

class PIDController {
 public:
  PIDController(const double &kp, const double &ki, const double &kd)
      : kp_(kp),
        ki_(ki),
        kd_(kd),
        previous_error_(0),
        previous_output_(0),
        integral_(0),
        first_hit_(true) {}
  ~PIDController() = default;

  void reset() {
    previous_error_ = 0.0;
    previous_output_ = 0.0;
    integral_ = 0.0;
    first_hit_ = true;
  }

  /**
   * @brief compute control value based on the error
   * @param error error value, the difference between
   * a desired value and a measured value
   * @param dt sampling time interval
   * @return control value based on PID terms
   */
  double control(const double &error, const double &dt) {
    if (dt <= 0) {
      return previous_output_;
    }
    double diff = 0;
    double output = 0;

    if (first_hit_)  // first_hit_: 用来选择是否计算diff
    {
      first_hit_ = false;
    } else {
      diff = (error - previous_error_) / dt;
    }

    integral_ += ki_ * error * dt;  // 积分环节

    output = kp_ * error + integral_ + diff * kd_;
    previous_output_ = output;
    previous_error_ = error;
    return output;
  }

 protected:
  double kp_ = 0.0;
  double ki_ = 0.0;
  double kd_ = 0.0;
  double previous_error_ = 0.0;
  double previous_output_ = 0.0;
  double integral_ = 0.0;
  bool first_hit_ = false;
};
}  // namespace minco_local_planner::utils
#endif /* __PID_CONTROL_H__ */
