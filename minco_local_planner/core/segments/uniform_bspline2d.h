/**
 * @Author: Yunkai Xia
 * @Date:   2023-03-02 13:09:05
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-07-13 21:13:39
 */
#include <stdint.h>

#ifndef __UNIFORM_BSPLINE2D_H__
#define __UNIFORM_BSPLINE2D_H__
#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>

#include "basis/Logger.h"
#include "basis/data_type.h"
#include "basis/math.hpp"
#include "basis/trajectory.h"
using namespace basis;
namespace parametric_curve {
// 参考 https://github.com/HKUST-Aerial-Robotics/Fast-Planner
// 根据实际的应用进行修改，更改数据格式以及维度
// 2维均匀B样条

class UniformBspline2d {
 private:
  // 关于B样条形状的控制
  MatD2f control_points_;  // B样条曲线的控制点 Nx2
  VecDf u_;                // knots vector 向量
  int p_;                  // 样条曲线的次数，p+1表示阶
  int n_;                  // n+1表示控制点数
  int m_;                  // m+1表示节点数

  decimal_t interval_;  // 节点之间的时间间隔
  // B样条曲线的物理约束
  decimal_t limit_vel_;
  decimal_t limit_acc_;
  decimal_t feasibility_tolerance_;
  decimal_t limit_ratio_;
  bool is_initialized_{false};

 public:
  UniformBspline2d() {}
  //  默认使用四阶三次B样条
  UniformBspline2d(const MatD2f &points, const decimal_t &interval,
                   const int &order = 3) {
    setUniformBspline(points, interval, order);
  }

  ~UniformBspline2d() {}

  // 设置控制点，次数，和时间间隔，得到B样条曲线
  void setUniformBspline(const MatD2f &points, const decimal_t &interval,
                         const int &order = 3) {
    // 设置控制点
    control_points_ = points;
    p_ = order;  // 次数
    interval_ = interval;

    n_ = points.rows() - 1;    // n+1表示控制点的数量
    m_ = n_ + p_ + 1;          // 节点向量数量
    u_ = VecDf::Zero(m_ + 1);  // 节点表

    // 均匀B样条的间隔相同
    for (int i = 0; i <= m_; ++i) {
      // 在有效的区间之前为负数
      if (i <= p_) {
        u_(i) = decimal_t(-p_ + i) * interval_;
      } else if (i > p_ && i <= m_ - p_) {
        u_(i) = u_(i - 1) + interval_;
      } else if (i > m_ - p_) {
        u_(i) = u_(i - 1) + interval_;
      }
    }
    is_initialized_ = true;
  }

  // 设置离散的路径，设置起点终点的速度与加速度，得到B样条曲线的控制点
  // 利用B样条曲线与多项式曲线的转换，拟合多项式，得到B样条曲线的控制点
  // 默认使用4阶三次B样条曲线,进行B样条曲线拟合
  //
  static bool parameterizeToBspline(const decimal_t &ts, const Path &point_set,
                                    const vec_Vec2f &start_end_derivative,
                                    MatD2f &ctrl_pts) {
    // 检测时间检测
    if (ts <= 0) {
      XLOG_ERROR("[B-spline]:time step error.");
      return false;
    }
    // 检测控制点的数量，点到数量不能少于次数
    if (point_set.size() <= 3) {
      XLOG_ERROR("[B-spline]:point set have only {} points.", point_set.size());
      return false;
    }

    // 检测起点终点约束
    if (start_end_derivative.size() != 4) {
      XLOG_ERROR("[B-spline]:derivatives error.");
      std::cout << "[B-spline]:derivatives error." << std::endl;
      return false;
    }
    // 中间点的数量
    const int K = point_set.size();
    XLOG_TRACE("points size is {}", K);

    // write A
    Eigen::Vector3d prow(3), vrow(3), arow(3);
    prow << 1, 4, 1;
    vrow << -1, 0, 1;
    arow << 1, -2, 1;

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(K + 4, K + 2);
    // 位置约束
    for (int i = 0; i < K; ++i)
      A.block(i, i, 1, 3) = (1 / 6.0) * prow.transpose();
    // 速度约束
    A.block(K, 0, 1, 3) = (1 / 2.0 / ts) * vrow.transpose();
    A.block(K + 1, K - 1, 1, 3) = (1 / 2.0 / ts) * vrow.transpose();
    // 加速度约束
    A.block(K + 2, 0, 1, 3) = (1 / ts / ts) * arow.transpose();
    A.block(K + 3, K - 1, 1, 3) = (1 / ts / ts) * arow.transpose();

    Eigen::VectorXd bx(K + 4), by(K + 4);
    // 约束的位置
    for (int i = 0; i < K; ++i) {
      bx(i) = point_set[i](0);
      by(i) = point_set[i](1);
    }

    for (int i = 0; i < 4; ++i) {
      bx(K + i) = start_end_derivative[i](0);
      by(K + i) = start_end_derivative[i](1);
    }

    // solve Ax = b
    Eigen::VectorXd px = A.colPivHouseholderQr().solve(bx);
    Eigen::VectorXd py = A.colPivHouseholderQr().solve(by);
    ctrl_pts.resize(K + 2, 2);
    ctrl_pts.col(0) = px;
    ctrl_pts.col(1) = py;
    return true;
  }

  TrajectoryPoint calcTrajPt(const Vec2f &pose, const Vec2f &vel,
                             const Vec2f &acc) {
    TrajectoryPoint traj_pt;
    traj_pt.setPos(pose);
    traj_pt.setVel(vel.norm());
    const decimal_t dx = vel.x();
    const decimal_t dy = vel.y();
    const decimal_t ddx = acc.x();
    const decimal_t ddy = acc.y();
    const decimal_t theta = normalizeAngleRad(atan2(dy, dx));  // theta
    const decimal_t kappa =
        (dx * ddy - dy * ddx) / pow((dx * dx + dy * dy), 1.5);  // kappa
    traj_pt.setKappa(kappa);
    traj_pt.setTheta(theta);

    return traj_pt;
  }

  TrajectoryPoint calcTrajPt(const Vec2f &pose, const Vec2f &vel,
                             const Vec2f &acc, const decimal_t &speed) {
    TrajectoryPoint traj_pt;
    traj_pt.setPos(pose);
    traj_pt.setVel(speed);
    const decimal_t dx = vel.x();
    const decimal_t dy = vel.y();
    const decimal_t ddx = acc.x();
    const decimal_t ddy = acc.y();
    const decimal_t theta = normalizeAngleRad(atan2(dy, dx));  // theta
    const decimal_t kappa =
        (dx * ddy - dy * ddx) / pow((dx * dx + dy * dy), 1.5);  // kappa
    traj_pt.setKappa(kappa);
    traj_pt.setTheta(theta);

    return traj_pt;
  }
// 使用梯形曲线进行速度规划
  void getTrapezoidTrajectory(Trajectory &traj_out, const decimal_t &start_vel,
                              const decimal_t &target_vel,
                              const decimal_t &end_vel, const decimal_t &acc,
                              const decimal_t &dec, const decimal_t &creep_dist,
                              const decimal_t &creep_speed,
                              const decimal_t &dt = 0.01) {
    if (!is_initialized_) {
      return;
    }
    decimal_t sa = 0, sc = 0, sd = 0, se = 0, s0 = 0;

    decimal_t target_speed = target_vel;
    decimal_t init_speed = start_vel;
    decimal_t distance = getLength();

    decimal_t end_speed = end_vel;
    // 设置初始速度
    if (init_speed < 0.05) {
      init_speed = 0.05;
      if (target_speed < init_speed) {
        init_speed = target_speed;
      }
    }

    if (end_speed <= creep_speed) {
      se = creep_dist;
      if (target_speed < end_speed) {
        end_speed = target_speed;
      }
    }
    sa = fabs(target_speed * (target_speed)-init_speed * init_speed) /
         (2.0 * acc);
    sd =
        fabs(target_speed * (target_speed)-end_speed * end_speed) / (2.0 * dec);

    if (sa + sd + se + s0 <= distance) {
      sc = distance - sa - sd - se - s0;
    } else {
      if (init_speed > target_speed && end_speed > target_speed) {
        target_speed =
            sqrt((init_speed * init_speed + end_speed * end_speed -
                  acc * (distance - se - s0) - dec * (distance - se - s0)) /
                 2.0);
      } else if (init_speed < target_speed && end_speed < target_speed) {
        target_speed =
            sqrt((acc * (distance - se - s0) + dec * (distance - se - s0) +
                  init_speed * init_speed + end_speed * end_speed) /
                 2.0);
      } else {
        target_speed =
            sqrt((init_speed * init_speed + end_speed * end_speed) / 2.0);
      }

      sa = fabs(target_speed * (target_speed)-init_speed * init_speed) /
           (2.0 * acc);
      sd = fabs(target_speed * (target_speed)-end_speed * end_speed) /
           (2.0 * dec);
      sc = 0;
    }

    traj_out.clear();
    // 生成轨迹点的匿名函数
    const UniformBspline2d vel_traj = getDerivative();
    const UniformBspline2d acc_traj = getDerivative().getDerivative();

    decimal_t tm{0}, tmp{0};
    getTimeSpan(tm, tmp);
    decimal_t s = 0;
    decimal_t speed = 0;
    for (decimal_t t = tm; t <= tmp; t += dt) {
      const Vec2f cur_pose = evaluateDeBoor(t);
      const Vec2f vel_2f = vel_traj.evaluateDeBoor(t);
      const Vec2f acc_2f = acc_traj.evaluateDeBoor(t);

      if (t == tm) {
        s = 0;
      } else {
        const Vec2f last_pose = traj_out.back().getPos();
        const decimal_t dis = (last_pose - cur_pose).norm();
        s += dis;
      }

      if (s < s0) {
        speed = init_speed;
      }

      else if ((s >= s0) && (s <= sa + s0))  // 加速阶段
      {
        speed = sqrt(2 * s * acc + init_speed * init_speed);

        if (target_speed < init_speed) {
          decimal_t speed2 = init_speed * init_speed - 2 * s * acc;
          if (speed2 < 0)
            speed = target_speed;
          else
            speed = sqrt(speed2);
        }
        XLOG_DEBUG("加速 dist {}", s);
      } else if ((s > sa + s0) && (s <= (sa + sc + s0)))  // 匀速阶段
      {
        speed = target_speed;
        XLOG_DEBUG("匀速 dist {}", s);
      } else if ((s > (sa + sc + s0)) &&
                 (s <= (sa + sc + sd + s0)))  // 减速阶段
      {
        decimal_t bb = s - sa - sc - s0;
        decimal_t speed2 = -2 * bb * dec + target_speed * target_speed;
        if (speed2 < 0)
          speed = end_speed;
        else
          speed = sqrt(speed2);

        if (target_speed < end_speed) {
          speed = sqrt(2 * bb * dec + target_speed * target_speed);
        }
        XLOG_DEBUG("减速 dist {}", s);
      } else if ((s > (sa + sc + sd + s0)) &&
                 (s <= (sa + sc + sd + se + s0)))  // 蠕行阶段
      {
        speed = end_speed;
        XLOG_DEBUG("蠕行 dist {}", s);
      } else {
        XLOG_DEBUG(
            "calculateTrapezoidParamters else....................... {} ", s);
      }
      XLOG_DEBUG("calculateTrapezoidParamters speed is {}", speed);
      TrajectoryPoint traj_pt = calcTrajPt(cur_pose, vel_2f, acc_2f, speed);
      traj_pt.setS(s);
      traj_out.emplace_back(traj_pt);
    }
  }

  void getTrajectory(Trajectory &traj_out, const decimal_t &dt = 0.01) {
    if (!is_initialized_) {
      return;
    }
    traj_out.clear();
    // 生成轨迹点的匿名函数
    const UniformBspline2d vel_traj = getDerivative();
    const UniformBspline2d acc_traj = getDerivative().getDerivative();
    decimal_t tm{0}, tmp{0};
    getTimeSpan(tm, tmp);
    decimal_t s = 0;
    for (decimal_t t = tm; t <= tmp; t += dt) {
      const Vec2f cur_pose = evaluateDeBoor(t);
      const Vec2f vel = vel_traj.evaluateDeBoor(t);
      const Vec2f acc = acc_traj.evaluateDeBoor(t);
      TrajectoryPoint traj_pt = calcTrajPt(cur_pose, vel, acc);
      if (t == tm) {
        traj_pt.setS(s);
      } else {
        const Vec2f last_pose = traj_out.back().getPos();
        const decimal_t dis = (last_pose - cur_pose).norm();
        s += dis;
        traj_pt.setS(s);
      }
      traj_out.emplace_back(traj_pt);
    }
  }

  int getOrder() const { return p_; }

  void setKnot(const VecDf &knot) { u_ = knot; }

  VecDf getKnot() const { return u_; }

  MatD2f getControlPoints() const { return control_points_; }

  MatD2f getDerivativeControlPoints() {
    // The derivative of a b-spline is also a b-spline, its order become p_-1
    // control point Qi = p_*(Pi+1-Pi)/(ui+p_+1-ui+1)
    MatD2f ctp(control_points_.rows() - 1, control_points_.cols());
    for (int i = 0; i < ctp.rows(); ++i) {
      ctp.row(i) = p_ * (control_points_.row(i + 1) - control_points_.row(i)) /
                   (u_(i + p_ + 1) - u_(i + 1));
    }
    return ctp;
  }

  decimal_t getInterval() const { return interval_; }

  bool getTimeSpan(decimal_t &um, decimal_t &um_p) const {
    if (p_ > u_.rows() || m_ - p_ > u_.rows()) return false;
    // 起点时间，
    um = u_(p_);
    // 终点时间
    um_p = u_(m_ - p_);

    return true;
  }
  // 通过de bool-cox计算B样条曲线上的值
  // u的范围[u_k-1, u_n+1]
  Vec2f evaluateDeBoor(const decimal_t &u) const {
    decimal_t ub = std::min(std::max(u_(p_), u), u_(m_ - p_));
    int k = p_;

    while (true) {
      if (u_(k + 1) >= ub) break;
      ++k;
    }

    vec_Vec2f d;
    // p+1个点
    for (int i = 0; i <= p_; ++i) {
      d.emplace_back(control_points_.row(k - p_ + i));
    }

    for (int r = 1; r <= p_; ++r) {
      for (int i = p_; i >= r; --i) {
        decimal_t alpha =
            (ub - u_[i + k - p_]) / (u_[i + 1 + k - r] - u_[i + k - p_]);
        // cout << "alpha: " << alpha << endl;
        d[i] = (1 - alpha) * d[i - 1] + alpha * d[i];
      }
    }

    return d[p_];
  }

  inline Vec2f evaluateDeBoorT(const decimal_t &t) const {
    return evaluateDeBoor(t + u_(p_));
  }

  UniformBspline2d getDerivative() {
    MatD2f ctp = getDerivativeControlPoints();
    UniformBspline2d derivative(ctp, interval_, p_ - 1);

    /* cut the first and last knot */
    Eigen::VectorXd knot(u_.rows() - 2);
    knot = u_.segment(1, u_.rows() - 2);
    derivative.setKnot(knot);

    return derivative;
  }

  /* check feasibility, adjust time */

  void setPhysicalLimits(const decimal_t &vel, const decimal_t &acc,
                         const decimal_t &tolerance) {
    limit_vel_ = vel;
    limit_acc_ = acc;
    limit_ratio_ = 1.1;
    feasibility_tolerance_ = tolerance;
    // std::cout << " limit_vel_  is " << limit_vel_ << std::endl;
    // std::cout << " limit_acc_  is " << limit_acc_ << std::endl;
    // std::cout << " feasibility_tolerance_  is " << feasibility_tolerance_
    //           << std::endl;
  }

  bool checkFeasibility(decimal_t &ratio, bool show = false) {
    bool fea = true;

    MatD2f P = control_points_;
    int dimension = control_points_.cols();

    /* check vel feasibility and insert points */
    decimal_t max_vel = -1.0;
    decimal_t enlarged_vel_lim =
        limit_vel_ * (1.0 + feasibility_tolerance_) + 1e-4;
    for (int i = 0; i < P.rows() - 1; ++i) {
      Eigen::VectorXd vel =
          p_ * (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1));

      if (fabs(vel(0)) > enlarged_vel_lim || fabs(vel(1)) > enlarged_vel_lim) {
        if (show)
          std::cout << "[Check]: Infeasible vel " << i << " :"
                    << vel.transpose() << std::endl;
        fea = false;

        for (int j = 0; j < dimension; ++j) {
          max_vel = std::max(max_vel, fabs(vel(j)));
        }
      }
    }

    /* acc feasibility */
    decimal_t max_acc = -1.0;
    decimal_t enlarged_acc_lim =
        limit_acc_ * (1.0 + feasibility_tolerance_) + 1e-4;
    for (int i = 0; i < P.rows() - 2; ++i) {
      Vec2f acc =
          p_ * (p_ - 1) *
          ((P.row(i + 2) - P.row(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
           (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
          (u_(i + p_ + 1) - u_(i + 2));

      if (fabs(acc(0)) > enlarged_acc_lim || fabs(acc(1)) > enlarged_acc_lim) {
        if (show)
          std::cout << "[Check]: Infeasible acc " << i << " :"
                    << acc.transpose() << std::endl;
        fea = false;

        for (int j = 0; j < dimension; ++j) {
          max_acc = std::max(max_acc, fabs(acc(j)));
        }
      }
    }

    ratio = std::max(max_vel / limit_vel_, sqrt(fabs(max_acc) / limit_acc_));

    return fea;
  }

  bool reallocateTime(bool show = false) {
    bool fea = true;

    Eigen::MatrixXd P = control_points_;
    int dimension = control_points_.cols();

    decimal_t max_vel, max_acc;

    /* check vel feasibility and insert points */
    for (int i = 0; i < P.rows() - 1; ++i) {
      Vec2f vel = p_ * (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1));

      if (fabs(vel(0)) > limit_vel_ + 1e-4 ||
          fabs(vel(1)) > limit_vel_ + 1e-4) {
        fea = false;
        if (show)
          std::cout << "[Realloc]: Infeasible vel " << i << " :"
                    << vel.transpose() << std::endl;

        max_vel = -1.0;
        for (int j = 0; j < dimension; ++j) {
          max_vel = std::max(max_vel, fabs(vel(j)));
        }

        decimal_t ratio = max_vel / limit_vel_ + 1e-4;
        if (ratio > limit_ratio_) ratio = limit_ratio_;

        decimal_t time_ori = u_(i + p_ + 1) - u_(i + 1);
        decimal_t time_new = ratio * time_ori;
        decimal_t delta_t = time_new - time_ori;
        decimal_t t_inc = delta_t / decimal_t(p_);

        for (int j = i + 2; j <= i + p_ + 1; ++j) {
          u_(j) += decimal_t(j - i - 1) * t_inc;
          if (j <= 5 && j >= 1) {
            // cout << "vel j: " << j << endl;
          }
        }

        for (int j = i + p_ + 2; j < u_.rows(); ++j) {
          u_(j) += delta_t;
        }
      }
    }

    /* acc feasibility */
    for (int i = 0; i < P.rows() - 2; ++i) {
      Eigen::VectorXd acc =
          p_ * (p_ - 1) *
          ((P.row(i + 2) - P.row(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
           (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
          (u_(i + p_ + 1) - u_(i + 2));

      if (fabs(acc(0)) > limit_acc_ + 1e-4 ||
          fabs(acc(1)) > limit_acc_ + 1e-4) {
        fea = false;
        if (show)
          std::cout << "[Realloc]: Infeasible acc " << i << " :"
                    << acc.transpose() << std::endl;

        max_acc = -1.0;
        for (int j = 0; j < dimension; ++j) {
          max_acc = std::max(max_acc, fabs(acc(j)));
        }

        decimal_t ratio = sqrt(max_acc / limit_acc_) + 1e-4;
        if (ratio > limit_ratio_) ratio = limit_ratio_;
        // cout << "ratio: " << ratio << endl;

        decimal_t time_ori = u_(i + p_ + 1) - u_(i + 2);
        decimal_t time_new = ratio * time_ori;
        decimal_t delta_t = time_new - time_ori;
        decimal_t t_inc = delta_t / decimal_t(p_ - 1);

        if (i == 1 || i == 2) {
          // cout << "acc i: " << i << endl;
          for (int j = 2; j <= 5; ++j) {
            u_(j) += decimal_t(j - 1) * t_inc;
          }

          for (int j = 6; j < u_.rows(); ++j) {
            u_(j) += 4.0 * t_inc;
          }
        } else {
          for (int j = i + 3; j <= i + p_ + 1; ++j) {
            u_(j) += decimal_t(j - i - 2) * t_inc;
            if (j <= 5 && j >= 1) {
              // cout << "acc j: " << j << endl;
            }
          }

          for (int j = i + p_ + 2; j < u_.rows(); ++j) {
            u_(j) += delta_t;
          }
        }
      }
    }

    return fea;
  }
  void lengthenTime(const decimal_t &ratio) {
    int num1 = 5;
    int num2 = getKnot().rows() - 1 - 5;

    decimal_t delta_t = (ratio - 1.0) * (u_(num2) - u_(num1));
    decimal_t t_inc = delta_t / decimal_t(num2 - num1);
    for (int i = num1 + 1; i <= num2; ++i) u_(i) += decimal_t(i - num1) * t_inc;
    for (int i = num2 + 1; i < u_.rows(); ++i) u_(i) += delta_t;
  }

  /* for performance evaluation */

  decimal_t getTimeSum() {
    decimal_t tm, tmp;
    getTimeSpan(tm, tmp);
    return tmp - tm;
  }
  decimal_t getLength(const decimal_t &res = 0.01) {
    decimal_t length = 0.0;
    decimal_t dur = getTimeSum();
    Vec2f p_l = evaluateDeBoorT(0.0), p_n;
    for (decimal_t t = res; t <= dur + 1e-4; t += res) {
      p_n = evaluateDeBoorT(t);
      length += (p_n - p_l).norm();
      p_l = p_n;
    }
    return length;
  }
  decimal_t getJerk() {
    UniformBspline2d jerk_traj =
        getDerivative().getDerivative().getDerivative();

    VecDf times = jerk_traj.getKnot();
    MatD2f ctrl_pts = jerk_traj.getControlPoints();
    int dimension = ctrl_pts.cols();

    decimal_t jerk = 0.0;
    for (int i = 0; i < ctrl_pts.rows(); ++i) {
      for (int j = 0; j < dimension; ++j) {
        jerk += (times(i + 1) - times(i)) * ctrl_pts(i, j) * ctrl_pts(i, j);
      }
    }

    return jerk;
  }
  void getMeanAndMaxVel(decimal_t &mean_v, decimal_t &max_v) {
    if (!is_initialized_) {
      return;
    }
    UniformBspline2d vel = getDerivative();
    decimal_t tm, tmp;
    vel.getTimeSpan(tm, tmp);

    decimal_t max_vel = -1.0, mean_vel = 0.0;
    int num = 0;
    for (decimal_t t = tm; t <= tmp; t += 0.01) {
      Vec2f vxd = vel.evaluateDeBoor(t);
      decimal_t vn = vxd.norm();

      mean_vel += vn;
      ++num;
      if (vn > max_vel) {
        max_vel = vn;
      }
    }

    mean_vel = mean_vel / decimal_t(num);
    mean_v = mean_vel;
    max_v = max_vel;
  }
  //
  void getMeanAndMaxAcc(decimal_t &mean_a, decimal_t &max_a) {
    UniformBspline2d acc = getDerivative().getDerivative();
    decimal_t tm, tmp;
    acc.getTimeSpan(tm, tmp);

    decimal_t max_acc = -1.0, mean_acc = 0.0;
    int num = 0;
    for (decimal_t t = tm; t <= tmp; t += 0.01) {
      Vec2f axd = acc.evaluateDeBoor(t);
      decimal_t an = axd.norm();

      mean_acc += an;
      ++num;
      if (an > max_acc) {
        max_acc = an;
      }
    }

    mean_acc = mean_acc / decimal_t(num);
    mean_a = mean_acc;
    max_a = max_acc;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace parametric_curve

#endif /* __UNIFORM_BSPLINE2D_H__ */
