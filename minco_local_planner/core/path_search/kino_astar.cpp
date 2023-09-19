/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-30 14:18:36
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-09-04 16:04:01
 */
#include "kino_astar.h"

#include <iostream>
using namespace std;
#include "kino_astar.h"
#include "module_manager/module_manager.h"
namespace minco_local_planner::path_search {

KinoAstar::KinoAstar() : PathSearch("KinoAstar") {}

bool KinoAstar::Init() { return true; }
bool KinoAstar::Start() { return true; }
void KinoAstar::Stop() {}

int KinoAstar::Search(const VehiclePose& start_pos, const VehiclePose& end_pos,
                      const Vec2d& init_ctrl) {
  if (!map_ptr_) {
    return NO_MAP;
  }

  return NO_PATH;
}

void KinoAstar::Reset() { std::cout << "reset " << std::endl; }
void KinoAstar::GetPath2D(Path2d& path) {}

double KinoAstar::EstimateHeuristic(const VehiclePose& currt_pt,
                                    const VehiclePose& target_pt,
                                    double& optimal_time) {
  const double w_time = 10;
  const Vec2d dp = target_pt.GetPos() - currt_pt.GetPos();
  // 当前速度
  const Vec2d v0 = currt_pt.GetVel();
  const Vec2d v1 = target_pt.GetVel();
  //   对启发函数进行时间求导，得到关于时间的四次多项式，
  double c1 = -36 * dp.dot(dp);
  double c2 = 24 * (v0 + v1).dot(dp);
  double c3 = -4 * (v0.dot(v0) + v0.dot(v1) + v1.dot(v1));
  double c4 = 0;
  double c5 = w_time;  // 时间花费的权重
  // 关于时间的一元四次方程是通过费拉里方法求解的，需要嵌套一个元三次方程进行求解，也就是代码中应的cubic（）函数
  Vec_d ts = Quartic(c5, c4, c3, c2, c1);
  double v_max = cfg_->max_vel * 0.5;  // 1.5表示最大速度
  double t_bar = (v0 - v1).lpNorm<Eigen::Infinity>() / v_max;
  ts.emplace_back(t_bar);
  double cost = 100000000;
  double t_d = t_bar;
  for (auto t : ts) {
    if (t < t_bar) continue;
    double c = -c1 / (3 * t * t * t) - c2 / (2 * t * t) - c3 / t + w_time * t;
    if (c < cost) {
      cost = c;
      t_d = t;
    }
  }
  optimal_time = t_d;
  // return 1.0 * (1 + 0.001) * cost;
  return cost;
}
Vec_d KinoAstar::Quartic(const double& a, const double& b, const double& c,
                         const double& d, const double& e) {
  Vec_d dts;

  double a3 = b / a;
  double a2 = c / a;
  double a1 = d / a;
  double a0 = e / a;

  Vec_d ys =
      Cubic(1, -a2, a1 * a3 - 4 * a0, 4 * a2 * a0 - a1 * a1 - a3 * a3 * a0);
  double y1 = ys.front();
  double r = a3 * a3 / 4 - a2 + y1;
  if (r < 0) return dts;

  double R = sqrt(r);
  double D, E;
  if (R != 0) {
    D = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 +
             0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
    E = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 -
             0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
  } else {
    D = sqrt(0.75 * a3 * a3 - 2 * a2 + 2 * sqrt(y1 * y1 - 4 * a0));
    E = sqrt(0.75 * a3 * a3 - 2 * a2 - 2 * sqrt(y1 * y1 - 4 * a0));
  }

  if (!std::isnan(D)) {
    dts.emplace_back(-a3 / 4 + R / 2 + D / 2);
    dts.emplace_back(-a3 / 4 + R / 2 - D / 2);
  }
  if (!std::isnan(E)) {
    dts.emplace_back(-a3 / 4 - R / 2 + E / 2);
    dts.emplace_back(-a3 / 4 - R / 2 - E / 2);
  }

  return dts;
}
Vec_d KinoAstar::Cubic(const double& a, const double& b, const double& c,
                       const double& d) {
  Vec_d dts;

  double a2 = b / a;
  double a1 = c / a;
  double a0 = d / a;

  double Q = (3 * a1 - a2 * a2) / 9;
  double R = (9 * a1 * a2 - 27 * a0 - 2 * a2 * a2 * a2) / 54;
  double D = Q * Q * Q + R * R;
  if (D > 0) {
    double S = std::cbrt(R + sqrt(D));
    double T = std::cbrt(R - sqrt(D));
    dts.emplace_back(-a2 / 3 + (S + T));
    return dts;
  } else if (D == 0) {
    double S = std::cbrt(R);
    dts.emplace_back(-a2 / 3 + S + S);
    dts.emplace_back(-a2 / 3 - S);
    return dts;
  } else {
    double theta = acos(R / sqrt(-Q * Q * Q));
    dts.emplace_back(2 * sqrt(-Q) * cos(theta / 3) - a2 / 3);
    dts.emplace_back(2 * sqrt(-Q) * cos((theta + 2 * M_PI) / 3) - a2 / 3);
    dts.emplace_back(2 * sqrt(-Q) * cos((theta + 4 * M_PI) / 3) - a2 / 3);
    return dts;
  }
}

bool KinoAstar::ComputeShotTraj(const VehiclePose& state1,
                                const VehiclePose& state2,
                                const double& time_to_goal) {
  const Vec2d p0 = state1.GetPos();
  const Vec2d dp = state2.GetPos() - p0;  // 起点与终点向量
  const Vec2d v0{0, 0};
  const Vec2d v1 = state2.GetVel();
  const Vec2d dv = v1 - v0;
  double t_d = time_to_goal;
  Eigen::MatrixXd coef(2, 4);
  Eigen::MatrixXd coef_vel(2, 4);
  Eigen::MatrixXd coef_acc(2, 4);
  end_vel_ = v1;
  Vec2d a =
      1.0 / 6.0 *
      (-12.0 / (t_d * t_d * t_d) * (dp - v0 * t_d) + 6 / (t_d * t_d) * dv);
  Vec2d b = 0.5 * (6.0 / (t_d * t_d) * (dp - v0 * t_d) - 2 / t_d * dv);
  Vec2d c = v0;
  Vec2d d = p0;
  // 1/6 * alpha * t^3 + 1/2 * beta * t^2 + v0
  // a*t^3 + b*t^2 + v0*t + p0
  coef.col(3) = a, coef.col(2) = b, coef.col(1) = c, coef.col(0) = d;
  Vec2d coord, vel, acc;
  Eigen::VectorXd poly1d, t, polyv, polya;
  Eigen::MatrixXd Tm(4, 4);
  Tm << 0, 1, 0, 0, 0, 0, 2, 0, 0, 0, 0, 3, 0, 0, 0, 0;
  /* ---------- forward checking of trajectory ---------- */
  double t_delta = t_d / 10;
  for (double time = t_delta; time <= t_d; time += t_delta) {
    t = Eigen::VectorXd::Zero(4);
    for (int j = 0; j < 4; j++) t(j) = pow(time, j);

    for (int dim = 0; dim < 2; dim++) {
      poly1d = coef.row(dim);
      polyv = (Tm * poly1d);
      polya = (Tm * Tm * poly1d);
      coord(dim) = poly1d.dot(t);
      vel(dim) = polyv.dot(t);
      acc(dim) = polya.dot(t);
      coef_vel.row(dim) = polyv;
      coef_acc.row(dim) = polya;
    }
    if (!map_ptr_->IsVerify(coord)) {
      LOG_WARN("coord is not Verify");
      return false;
    }
    if (map_ptr_->IsOccupied(coord)) {
      LOG_WARN("coord is ccupied");
      return false;
    }
  }
  coef_shot_ = coef;
  coef_shot_vel_ = coef_vel;
  coef_shot_acc_ = coef_acc;
  t_shot_ = t_d;
  is_shot_succ_ = true;
  return true;
}
void KinoAstar::StateTransit(VehiclePose& state0, VehiclePose& state1,
                             const Vec2d& um, const double& tau) {}
}  // namespace minco_local_planner::path_search
