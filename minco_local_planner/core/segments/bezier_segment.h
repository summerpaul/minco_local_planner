/**
 * @Author: Yunkai Xia
 * @Date:   2023-01-09 10:03:09
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-08-30 11:05:59
 */
#include <stdint.h>

#ifndef __BEZIER_SEGMENT_H__
#define __BEZIER_SEGMENT_H__
#include <iostream>

#include "basis/data_type.h"
#include "basis/logger.h"
#include "basis/math.h"
#include "basis/trajectory.h"
#include "bezier.h"
namespace minco_local_planner::segments {
using namespace basis;

class BezierSegment {
 public:
  BezierSegment() {}
  /**
   * @brief Construct a new Bezier Segment object
   *
   * @param start 起点的位置
   * @param end 终点的位置
   * @param offset 距离偏移量
   */
  BezierSegment(const TrajectoryPoint &start, const TrajectoryPoint &end,
                const double &offset = 2.0) {
    const double dist = (start - end).head(2).norm() / offset;
    const Vec2d p0 = start.head(2);
    const Vec2d p3 = end.head(2);
    Vec2d p1, p2;
    p1[0] = p0[0] + dist * cos(start[2]);
    p1[1] = p0[1] + dist * sin(start[2]);
    p2[0] = p3[0] - dist * cos(end[2]);
    p2[1] = p3[1] - dist * sin(end[2]);
    GenerateBezierSegment(p0, p1, p2, p3);
  }
  /**
   * @brief Construct a new Bezier Segment object
   *
   * @param control_points
   */
  BezierSegment(const vec_Vec2d &control_points) {
    const Vec2d p0 = control_points[0];
    const Vec2d p1 = control_points[1];
    const Vec2d p2 = control_points[2];
    const Vec2d p3 = control_points[3];
    GenerateBezierSegment(p0, p1, p2, p3);
  }
  /**
   * @brief Construct a new Bezier Segment object
   *
   * @param p0 控制点0
   * @param p1 控制点1
   * @param p2 控制点2
   * @param p3 控制点3
   */
  BezierSegment(const Vec2d &p0, const Vec2d &p1, const Vec2d &p2,
                const Vec2d &p3) {
    GenerateBezierSegment(p0, p1, p2, p3);
  }
  /**
   * @brief Construct a new Bezier Segment object
   *
   * @param control_points 矩阵表示的控制点
   */
  BezierSegment(const Mat4x2d &control_points) {
    const Vec2d p0 = control_points.row(0).transpose();
    const Vec2d p1 = control_points.row(1).transpose();
    const Vec2d p2 = control_points.row(2).transpose();
    const Vec2d p3 = control_points.row(3).transpose();
    GenerateBezierSegment(p0, p1, p2, p3);
  }

  /**
   * @brief 使用四个控制点，生成三阶Bezier曲线
   *
   * @param p0 控制点0
   * @param p1 控制点1
   * @param p2 控制点2
   * @param p3 控制点3
   */
  void GenerateBezierSegment(const Vec2d &p0, const Vec2d &p1, const Vec2d &p2,
                             const Vec2d &p3) {
    cubic_bezier_ = Bezier::Bezier<3>({
        {(float)p0[0], (float)p0[1]},
        {(float)p1[0], (float)p1[1]},
        {(float)p2[0], (float)p2[1]},
        {(float)p3[0], (float)p3[1]},
    });
    d_cubic_bezier_ = cubic_bezier_.derivative();
    dd_cubic_bezier_ = d_cubic_bezier_.derivative();
    control_points_.clear();
    control_points_.emplace_back(p0);
    control_points_.emplace_back(p1);
    control_points_.emplace_back(p2);
    control_points_.emplace_back(p3);

    control_points_mat_.row(0) = p0;
    control_points_mat_.row(1) = p1;
    control_points_mat_.row(2) = p2;
    control_points_mat_.row(3) = p3;

    start_pt_ = p0;
    end_pt_ = p3;
  }
  ~BezierSegment() {}
  /**
   * @brief Get the Control Points object 获取数据形式的Bezier曲线控制点
   *
   * @return vec_Vec2d Bezier曲线控制点
   */
  const Points2d &GetControlPoints() const { return control_points_; }

  void SetControlPoints(const vec_Vec2d &control_points) {
    const Vec2d p0 = control_points[0];
    const Vec2d p1 = control_points[1];
    const Vec2d p2 = control_points[2];
    const Vec2d p3 = control_points[3];
    GenerateBezierSegment(p0, p1, p2, p3);
  }

  /**
   * @brief Get the Control Points Mat object
   * 获取Eigen矩阵形式的Bezier曲线控制点
   *
   * @return Mat4x2d 矩阵形式的Bezier控制点
   */
  const Mat4x2d &GetControlPointsMat() const { return control_points_mat_; }

  /**
   * @brief Get the Point object
   *
   * @param t bezier参数
   * @return Vec2d 控制位置
   */
  Vec2d GetPathPoint(const double &t) const {
    Vec2d point;
    double t_tmp = t <= 0 ? 0 : t;
    t_tmp = t >= 1 ? 1 : t;
    point[0] = (double)cubic_bezier_.valueAt(t_tmp, 0);
    point[1] = (double)cubic_bezier_.valueAt(t_tmp, 1);
    return point;
  }

  /**
   * @brief Get the Pose object
   *
   * @param t
   * @return TrajectoryPoint 轨迹信息
   */
  TrajectoryPoint GetTrajectoryPoint(const double &t) const {
    TrajectoryPoint pose;
    double t_tmp = t <= 0 ? 0 : t;
    t_tmp = t >= 1 ? 1 : t;
    const double x = (double)cubic_bezier_.valueAt(t_tmp, 0);
    const double y = (double)cubic_bezier_.valueAt(t_tmp, 1);
    pose.SetX(x);
    pose.SetY(y);

    const double dx = (double)d_cubic_bezier_.valueAt(t_tmp, 0);
    const double dy = (double)d_cubic_bezier_.valueAt(t_tmp, 1);
    const double ddx = (double)dd_cubic_bezier_.valueAt(t_tmp, 0);
    const double ddy = (double)dd_cubic_bezier_.valueAt(t_tmp, 1);
    const double yaw = NormalizeAngleRad(atan2(dy, dx));  // theta
    const double kappa =
        (dx * ddy - dy * ddx) / pow((dx * dx + dy * dy), 1.5);  // kappa
    pose.SetYaw(yaw);
    pose.SetKappa(kappa);
    pose.SetVel(target_speed_);
    return pose;
  }

  /**
   * @brief Get the Boundary object 获取Bezier曲线的左右边界
   *
   * @param left_boundry Bezier曲线的左边界
   * @param right_boundary Bezier曲线的右边界
   * @param min_interval  Bezier曲线的离散点间隔
   * @param clear_traj 是否清空Bezier曲线
   */
  void GetBoundary(Path2d &left_boundry, Path2d &right_boundary,
                   const double &min_interval = 0.02,
                   const bool &clear_traj = false) const {
    if (clear_traj) {
      left_boundry.clear();
      right_boundary.clear();
    }
    const double dt = (min_interval / GetLength());
    for (double t = 0; t <= 1; t += dt) {
      const TrajectoryPoint pt = GetTrajectoryPoint(t);
      const double yaw = pt.GetYaw();
      const Vec2d center_pose = pt.GetPos();
      const Vec2d direction(std::cos(yaw), std::sin(yaw));
      Mat2d rotation_mat;
      rotation_mat << cos(M_PI_2), -sin(M_PI_2), sin(M_PI_2), cos(M_PI_2);
      const Vec2d left_norm = rotation_mat * direction;
      const Vec2d right_norm = rotation_mat.inverse() * direction;
      const Vec2d left_pose = center_pose + path_width_ * left_norm;
      const Vec2d right_pose = center_pose + path_width_ * right_norm;
      left_boundry.emplace_back(left_pose);
      right_boundary.emplace_back(right_pose);
    }
  }

  /**
   * @brief Get the Traj object 获取Bezier曲线的轨迹
   *
   * @param traj_out 输出的轨迹
   * @param min_interval 轨迹的距离间隔
   * @param clear_traj 是否清空轨迹
   * @param end_segment 是否最后一段轨迹
   */
  void GetTrajectory(Trajectory &traj_out, const double &min_interval = 0.02,
                     const bool &clear_traj = false,
                     bool end_segment = false) const {
    if (clear_traj) {
      traj_out.clear();
    }
    const double dt = (min_interval / GetLength());
    for (double t = 0; t < 1; t += dt) {
      traj_out.EmplaceBack(GetTrajectoryPoint(t));
    }
    if (end_segment) {
      traj_out.EmplaceBack(GetTrajectoryPoint(1.0));
    }
  }

  /**
   * @brief Get the Length object
   *
   * @return double bezier曲线的长度
   */
  double GetLength() const { return (double)cubic_bezier_.length(); }

  /**
   * @brief Get the Path object 获取Bezier曲线路径
   *
   * @param path_out
   * @param min_interval
   * @param clear_path
   */
  void GetPath(Path2d &path_out, const double &min_interval = 0.02,
               const bool &clear_path = false) const {
    if (clear_path) {
      path_out.clear();
    }
    const double dt = (min_interval / GetLength());
    for (double t = 0; t <= 1; t += dt) {
      path_out.emplace_back(GetPathPoint(t));
    }
  }
  /**
   * @brief Set the Width object 设置Bezier曲线段的路宽
   *
   * @param width
   */
  void SetWidth(const double &width) { path_width_ = width; }
  /**
   * @brief Set the Speed object 设置Bezier曲线的目标速度
   *
   * @param speed
   */
  void SetSpeed(const double &speed) { target_speed_ = speed; }

  void SetLocalPlanFlag(const bool &flag) { local_plan_flag_ = flag; }

  const bool SetLocalPlanFlag() const { return local_plan_flag_; }

  const double GetWidth() const { return path_width_; }
  const double GetSpeed() const { return target_speed_; }
  void SetStartId(const int &id) { start_id_ = id; }
  const int GetStartId() const { return start_id_; }

  void SetEndId(const int &id) { end_id_ = id; }
  const int GetEndId() const { return end_id_; }

  void SetId(const int &id) { lane_id_ = id; }
  const int GetId() const { return lane_id_; }

  const Vec2d GetEndPoint() const { return end_pt_; }
  const Vec2d GetStartPoint() const { return start_pt_; }

 private:
  Bezier::Bezier<3> cubic_bezier_;    // 表示位置信息的3阶bezier曲线
  Bezier::Bezier<2> d_cubic_bezier_;  // 表示速度信息的2阶Bezier曲线
  Bezier::Bezier<1> dd_cubic_bezier_;  // 表示加速度信息的1阶Bezier曲线
  vec_Vec2d control_points_;      // 用数组表示的Bezier曲线控制点
  Mat4x2d control_points_mat_;    // 用矩阵表示的Bezier曲线控制点
  double path_width_{1.0};        // Bezier曲线段的路宽
  double target_speed_{1.0};      // Bezier曲线的目标速度
  double local_plan_flag_{true};  // Bezier曲线是否启用局部规划
  int start_id_{-1};              // 路段中起点id
  int end_id_{-1};                // 路段中终点id
  int lane_id_{-1};               // 路段的id
  Vec2d start_pt_{Vec2d::Zero()};
  Vec2d end_pt_{Vec2d::Zero()};
};

// Bezier曲线段
class BezierSegments : public std::vector<BezierSegment> {
 public:
  /**
   * @brief Get the Length object 得到多段Bezier段的长度
   *
   * @return double 曲线段的长度
   */
  double GetLength() const {
    double l = 0;
    for (size_t i = 1; i < size(); i++) {
      l += (*this)[i].GetLength();
    }
    return l;
  }

  /**
   * @brief Get the Length List object 得到多段Bezier曲线的长度序列
   *
   * @return std::vector<double> 曲线长度数组
   */
  std::vector<double> GetLengthList() const {
    std::vector<double> length_list;
    for (size_t i = 1; i < size(); i++) {
      length_list.emplace_back((*this)[i].GetLength());
    }
    return length_list;
  }

  /**
   * @brief Get the Traj object 获取多段曲线段的轨迹
   *
   * @param traj_out  输出的轨迹
   * @param min_interval 轨迹的距离间隔
   */
  void GetTrajectory(Trajectory &traj_out,
                     const double &min_interval = 0.02) const {
    const size_t segment_size = size();
    if (segment_size == 0) {
      return;
    }
    for (size_t i = 0; i < segment_size; i++) {
      // 第一段需要清空轨迹
      if (i == 0 && i != segment_size - 1) {
        (*this)[i].GetTrajectory(traj_out, min_interval, true);
      }
      // 是第一段也是最后一段
      else if (i == 0 && i == segment_size - 1) {
        (*this)[i].GetTrajectory(traj_out, min_interval, true, true);
      }
      // 仅是最后一段
      else if (i == segment_size - 1 && i > 0) {
        (*this)[i].GetTrajectory(traj_out, min_interval, false, true);
      }
      // 普通的中间段
      else {
        (*this)[i].GetTrajectory(traj_out, min_interval, false);
      }
    }
  }

  /**
   * @brief Get the Path object 获取多段bezier的路径
   *
   * @param path_out 输出的路径
   * @param dt
   */
  void GetPath(Path2d &path_out, const double &dt = 0.02) const {
    const size_t segment_size = size();
    if (segment_size == 0) {
      return;
    }
    for (size_t i = 0; i < segment_size; i++) {
      if (i == 0)
        (*this)[i].GetPath(path_out, dt, true);
      else
        (*this)[i].GetPath(path_out, dt);
    }

    const Vec2d back_pose = path_out.back();
    const Vec2d end_pt = (*this)[segment_size - 1].GetPathPoint(1.0);
    if ((end_pt - back_pose).norm() > 0) {
      path_out.emplace_back(end_pt);
    }
  }

  /**
   * @brief Get the Boundary object 获取多段Bezier曲线的左右边界
   *
   * @param left_boundry 左边界
   * @param right_boundary 有边界
   * @param min_interval 距离间隔
   */
  void GetBoundary(Path2d &left_boundry, Path2d &right_boundary,
                   const double &min_interval = 0.02) const {
    const size_t segment_size = size();
    if (segment_size == 0) {
      return;
    }
    for (size_t i = 0; i < segment_size; i++) {
      if (i == 0)
        (*this)[i].GetBoundary(left_boundry, right_boundary, min_interval,
                               true);
      else
        (*this)[i].GetBoundary(left_boundry, right_boundary, min_interval);
    }
  }
  /**
   * @brief Get the All Control Points object 获取所有Bezier曲线的控制点
   *
   * @param all_control_points
   */
  void GetAllControlPoints(Points2d &all_control_points) const {
    const size_t segment_size = size();
    if (segment_size == 0) {
      return;
    }
    all_control_points.clear();
    for (size_t i = 0; i < segment_size; i++) {
      auto control_points = (*this)[i].GetControlPoints();
      for (auto &control_point : control_points) {
        all_control_points.emplace_back(control_point);
      }
    }
  }

};

}  // namespace minco_local_planner::segments

#endif /* __BEZIER_SEGMENT_H__ */
