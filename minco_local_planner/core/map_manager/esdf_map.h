/**
 * @Author: Yunkai Xia
 * @Date:   2023-09-05 08:48:04
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-09-05 10:58:46
 */
#include <stdint.h>

#ifndef __ESDF_MAP_H__
#define __ESDF_MAP_H__
#include "basis/pcl_types.h"
#include "grid_map.h"
namespace minco_local_planner::map_manager {

class ESDFMap : public GridMap {
 public:
  typedef std::shared_ptr<ESDFMap> Ptr;

 public:
  void GetESDFPointCloud(PointCloud3di &cloud) {
    if (!is_generate_esdf_) {
      return;
    }
    cloud.clear();
    double dist;
    pcl::PointXYZI pt;
    const double min_dist = 0.0;
    const double max_dist = 5.0;
    for (int x = 0; x < dim_[0]; x += 1) {
      for (int y = 0; y < dim_[1]; y += 1) {
        Vec2d pos = IntToDouble(Vec2i(x, y));
        dist = GetDistance(Vec2i(x, y));
        dist = std::min(dist, max_dist);
        dist = std::max(dist, min_dist);
        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = -0.1;
        pt.intensity = (dist - min_dist) / (max_dist - min_dist);

        cloud.emplace_back(pt);
        if (dist < 0) {
          std::cout << "dist is " << dist << std::endl;
        }
      }
    }
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = "odom";
  }

  void GenerateESDF2d() {
    if (!is_initialized_) {
      std::cout << "map is not is_initialized" << std::endl;
      return;
    }

    Vec_d grid_esdf_buffer(data_size_, 0);
    Vec_d distance_buffer(data_size_, 0);

    for (int x = 0; x < dim_[0]; x++) {
      FillESDF(
          [&](int y) {
            return IsOccupied(Vec2i(x, y)) == 1
                       ? 0
                       : std::numeric_limits<double>::max();
          },
          [&](int y, double val) {
            grid_esdf_buffer[GetIndex(Vec2i(x, y))] = val;
          },
          0, dim_[1] - 1, 1);
    }
    for (int y = 0; y < dim_[1]; y++) {
      FillESDF([&](int x) { return grid_esdf_buffer[GetIndex(Vec2i(x, y))]; },
               [&](int x, double val) {
                 distance_buffer[GetIndex(Vec2i(x, y))] = res_ * std::sqrt(val);
               },
               0, dim_[0] - 1, 0);
    }
    std::lock_guard<std::mutex> lock(distance_buffer_mutex_);
    distance_buffer_ = distance_buffer;
    is_generate_esdf_ = true;
  }

  double GetDistance(const Vec2i &pn) {
    if (!IsVerify(pn)) {
      return 0;
    }
    return distance_buffer_[GetIndex(pn)];
  }

  double GetSDFValue(const Vec2d &pt) {
    /* use Bilinear interpolation */
    // 寻找周围四个点，求出中间点的距离

    // 已知Q11,Q12, Q21, Q22点处的dist值，求解中间点P的距离与梯度
    //    Q12----_Q22
    //     |      |
    //     |   P  |
    //     |      |
    //    Q11----_Q21

    // https://handwiki.org/wiki/Bilinear_interpolation

    // 近似到栅格坐标
    const Vec2i P_index = DoubleToInt(pt);
    // 周围四个点的坐标
    const Vec2i Q11_index = P_index + Vec2i(-1, -1);
    const Vec2i Q21_index = P_index + Vec2i(1, -1);
    const Vec2i Q12_index = P_index + Vec2i(1, -1);
    const Vec2i Q22_index = P_index + Vec2i(1, 1);
    // 周围四个点的值
    const double f_Q11 = GetDistance(Q11_index);
    const double f_Q21 = GetDistance(Q21_index);
    const double f_Q12 = GetDistance(Q12_index);
    const double f_Q22 = GetDistance(Q22_index);

    // 转换成坐标进行插值
    const Vec2d Q11_pos = IntToDouble(Q11_index);
    // Vec2f Q21_pos = intToFloat(Q21_index);
    // Vec2f Q12_pos = intToFloat(Q12_index);
    const Vec2d Q22_pos = IntToDouble(Q22_index);

    const double x2_x1 = (Q22_pos - Q11_pos).x();
    const double y2_y1 = (Q22_pos - Q11_pos).y();
    const double x_x1 = (pt - Q11_pos).x();
    const double y_y1 = (pt - Q11_pos).y();
    const double x2_x = (Q22_pos - pt).x();
    const double y2_y = (Q22_pos - pt).y();

    const double f_R1 = x2_x / x2_x1 * f_Q11 + x_x1 / x2_x1 * f_Q21;
    const double f_R2 = x2_x / x2_x1 * f_Q12 + x_x1 / x2_x1 * f_Q22;
    const double f_P = y2_y / y2_y1 * f_R1 + y_y1 / y2_y1 * f_R2;
    return f_P;
  }

  void EvaluateEDTWithGrad(const Vec2d &pt, double &dist, Vec2d &grad) {
    // 已知Q11,Q12, Q21, Q22点处的dist值，求解中间点P的距离与梯度
    //    Q12----_Q22
    //     |      |
    //     |   P  |
    //     |      |
    //    Q11----_Q21
    const Vec2i P_index = DoubleToInt(pt);
    // 周围四个点的坐标
    const Vec2i Q11_index = P_index + Vec2i(-1, -1);
    const Vec2i Q21_index = P_index + Vec2i(1, -1);
    const Vec2i Q12_index = P_index + Vec2i(1, -1);
    const Vec2i Q22_index = P_index + Vec2i(1, 1);
    if (!IsVerify(Q11_index) || !IsVerify(Q21_index) || !IsVerify(Q12_index) ||
        !IsVerify(Q22_index)) {
      grad = Vec2d(0, 0);
      dist = GetDistance(P_index);
      return;
    }

    // 周围四个点的值
    const double f_Q11 = GetDistance(Q11_index);
    const double f_Q21 = GetDistance(Q21_index);
    const double f_Q12 = GetDistance(Q12_index);
    const double f_Q22 = GetDistance(Q22_index);

    // 转换成坐标进行插值
    const Vec2d Q11_pos = IntToDouble(Q11_index);
    const Vec2d Q21_pos = IntToDouble(Q21_index);
    const Vec2d Q12_pos = IntToDouble(Q12_index);
    const Vec2d Q22_pos = IntToDouble(Q22_index);

    const double x2_x1 = (Q22_pos - Q11_pos).x();
    const double y2_y1 = (Q22_pos - Q11_pos).y();
    const double x_x1 = (pt - Q11_pos).x();
    const double y_y1 = (pt - Q11_pos).y();
    const double x2_x = (Q22_pos - pt).x();
    const double y2_y = (Q22_pos - pt).y();

    const double f_R1 = x2_x / x2_x1 * f_Q11 + x_x1 / x2_x1 * f_Q21;
    const double f_R2 = x2_x / x2_x1 * f_Q12 + x_x1 / x2_x1 * f_Q22;
    const double f_P = y2_y / y2_y1 * f_R1 + y_y1 / y2_y1 * f_R2;

    dist = f_P;
    const double f_Q22_f_P = f_Q22 - f_P;
    const double f_Q21_f_P = f_Q21 - f_P;
    const double f_P_f_Q11 = f_P - f_Q11;
    const double f_P_f_Q12 = f_P - f_Q12;
    const Vec2d diff_Q22_P = Q22_pos - pt;
    const Vec2d diff_Q21_P = Q21_pos - pt;
    const Vec2d diff_P_Q12 = pt - Q12_pos;
    const Vec2d diff_P_Q11 = pt - Q11_pos;

    grad.x() = f_Q22_f_P / diff_Q22_P.x() + f_Q21_f_P / diff_Q21_P.x() +
               f_P_f_Q11 / diff_P_Q11.x() + f_P_f_Q12 / diff_P_Q12.x();
    grad.y() = f_Q22_f_P / diff_Q22_P.y() + f_Q21_f_P / diff_Q21_P.y() +
               f_P_f_Q11 / diff_P_Q11.y() + f_P_f_Q12 / diff_P_Q12.y();
    grad = 0.25 * grad;
  }

 private:
  template <typename F_get_val, typename F_set_val>
  void FillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end,
                int dim) {
    int v[dim_(dim)];
    double z[dim_(dim) + 1];
    int k = start;
    v[start] = start;
    z[start] = -std::numeric_limits<double>::max();
    z[start + 1] = std::numeric_limits<double>::max();
    for (int q = start + 1; q <= end; q++) {
      k++;
      double s;
      do {
        k--;
        s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) /
            (2 * q - 2 * v[k]);
      } while (s <= z[k]);
      k++;
      v[k] = q;
      z[k] = s;
      z[k + 1] = std::numeric_limits<double>::max();
    }
    k = start;
    for (int q = start; q <= end; q++) {
      while (z[k + 1] < q) k++;
      double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
      f_set_val(q, val);
    }
  }

 public:
 private:
  Vec_d distance_buffer_;
  bool is_generate_esdf_{false};
  std::mutex distance_buffer_mutex_;
};
}  // namespace minco_local_planner::map_manager

#endif /* __ESDF_MAP_H__ */
