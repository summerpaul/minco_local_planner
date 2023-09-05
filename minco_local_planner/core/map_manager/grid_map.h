/**
 * @Author: Xia Yunkai
 * @Date:   2023-08-24 21:20:40
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-09-05 08:50:26
 */
#include <stdint.h>

#ifndef __GRID_MAP_H__
#define __GRID_MAP_H__

#include <memory>
#include <mutex>
#include <string>

#include "basis/data_type.h"
#include "basis/rigid2d.h"
#include "raycast.h"
namespace minco_local_planner::map_manager {

using namespace basis;

const int8_t OCC = 100;     // 栅格占据
const int8_t FREE = 0;      // 栅格未被占据
const int8_t UNKNOWM = -1;  // 栅格状态未知

class GridMap {
 public:
  typedef std::shared_ptr<GridMap> Ptr;

 public:
  GridMap(const GridMap &) = default;
  GridMap &operator=(const GridMap &) = default;
  GridMap(GridMap &&) = default;
  GridMap &operator=(GridMap &&) = default;
  GridMap() = default;
  virtual ~GridMap() = default;

  GridMap(const std::string &name) : name_(name) {}

  GridMap(const Pose2d &origin, const Vec2i &dim,
          const std::vector<int8_t> &data, const double &res) {
    CreateGridMap(origin, dim, data, res);
  }

  void CreateGridMap(const Pose2d &origin, const Vec2i &dim,
                     const std::vector<int8_t> &data, const double &res) {
    mutex_.lock();
    res_ = res;
    res_inv_ = 1 / res;
    origin_ = origin;
    dim_ = dim;
    data_ = data;
    width_ = dim_.x();
    height_ = dim_.y();
    data_size_ = data.size();
    mutex_.unlock();
    is_initialized_ = true;
  }

  const double GetRes() const { return res_; }
  const Pose2d GetOrigin() const { return origin_; }
  const Vec2i GetDim() const { return dim_; }
  const std::vector<int8_t> &GetData() const { return data_; }
  size_t GetIndex(const Vec2i &pn) { return pn(0) + width_ * pn(1); }

  void SetDataZero() { data_.assign(data_size_, FREE); }

  void SetData(const std::vector<int8_t> &data) {
    if (data.size() != data_size_) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    data_ = data;
  }
  void SetOrigin(const Pose2d &origin) { origin_ = origin; }

  bool IsVerify(const Vec2i &pn) {
    bool x_ok = pn[0] >= 0 && pn[0] < width_;
    bool y_ok = pn[1] >= 0 && pn[1] < height_;
    bool size_ok = GetIndex(pn) < data_size_;
    return x_ok && y_ok && size_ok;
  }

  bool IsVerify(const Vec2d &pt) { return IsVerify(DoubleToInt(pt)); }

  bool IsFree(const size_t &idx) {
    if (idx > data_size_) {
      return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    return data_[idx] == FREE;
  }

  bool IsFree(const Vec2i &pn) { return IsFree(GetIndex(pn)); }

  bool IsFree(const Vec2d &pt) { return IsFree(DoubleToInt(pt)); }

  bool IsUnknown(const size_t &idx) {
    if (idx > data_size_) {
      return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    return data_[idx] == UNKNOWM;
  }

  bool IsUnknown(const Vec2i &pn) { return IsUnknown(GetIndex(pn)); }
  bool IsUnknown(const Vec2d &pt) { return IsUnknown(DoubleToInt(pt)); }

  bool IsOccupied(const size_t &idx) {
    if (idx > data_size_) {
      return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    return data_[idx] == OCC;
  }

  bool IsOccupied(const Vec2i &pn) { return IsOccupied(GetIndex(pn)); }
  bool IsOccupied(const Vec2d &pt) { return IsOccupied(DoubleToInt(pt)); }

  void CheckCollisionUsingPosAndYaw(const Pose2d &state,
                                    const Points2d &car_vertex, bool &res) {
    res = false;
    Vec2d pos = state.head(2);
    double yaw = state[2];
    Mat2d Rotation_matrix;
    Rotation_matrix << cos(yaw), -sin(yaw), sin(yaw), cos(yaw);
    for (int i = 0; i < 4; i++) {
      Vec2d start_point = pos + Rotation_matrix * car_vertex[i];
      Vec2d end_point = pos + Rotation_matrix * car_vertex[i + 1];
      RayCaster raycaster;
      bool need_ray =
          raycaster.SetInput(start_point * res_inv_, end_point * res_inv_);
      if (!need_ray) return;

      Vec2d half(0.5, 0.5);
      Vec2d ray_pt;
      while (raycaster.Step(ray_pt)) {
        Vec2d tmp = (ray_pt + half) * res_;
        if (IsOccupied(tmp)) {
          res = true;
          return;
        }
      }
    }
  }

  bool Query(const Vec2i &pn) {
    if (!IsVerify(pn)) {
      return false;
    }
    return (!IsOccupied(pn));
  }

  bool Query(const int &x, const int &y, const int range) {
    // #pragma omp parallel for num_threads(32)
    for (int i = -range; i < range + 1; i++) {
      for (int j = -range; j < range + 1; j++) {
        if (!Query(Vec2d(x + i, y + j))) {
          return false;
        }
      }
    }
    return true;
  }
  bool Query(const Vec2d &pt) { return Query(DoubleToInt(pt)); }

  void SetOccupied(const size_t &index) {
    if (index > data_size_) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    data_[index] = OCC;
  }
  // void SetOccupied(const Vec2d &pt) { SetOccupied(DoubleToInt(pt)); }
  void SetOccupied(const Vec2i &pn) {
    if (!IsVerify(pn)) {
      // std::cout << "IsVerify" << std::endl;
      return;
    }
    const int index = GetIndex(pn);
    if (index >= static_cast<int>(data_.size())) {
      std::cout << "error " << std::endl;
    }
    SetOccupied(index);
  }

  void SetInfOccupied(const Vec2d &pt, const double &inf_size = 0.2) {
    const int inf_step = ceil(inf_size / res_);
    const Vec2i pn = DoubleToInt(pt);
    if (inf_size == 0) {
      SetOccupied(pn);
    } else if (inf_size > 0) {
      for (int x = -inf_step; x < inf_step; ++x)
        for (int y = -inf_step; y < inf_step; ++y) {
          SetOccupied(pn + Vec2i(x, y));
        }
    }
  }

  Vec2i DoubleToInt(const Vec2d &pt) {
    // 全局坐标系转局部坐标系

    Vec2d point = SubVec2d(origin_, pt);
    Vec2i pn;
    pn[0] = std::round((point[0]) / res_ - 0.5);
    pn[1] = std::round((point[1]) / res_ - 0.5);

    return pn;
  }
  Vec2d IntToDouble(const Vec2i &pn) {
    Vec2d pt = (pn.template cast<double>() + Vec2d::Constant(0.5)) * res_;

    // 局部坐标系转全局坐标系
    return AddVec2d(origin_, pt);
  }

 protected:
  std::string name_;
  Pose2d origin_;
  Vec2i dim_;
  double res_;                // 栅格地图的分辨率
  double res_inv_;            // 栅格地图分辨率的倒数
  std::vector<int8_t> data_;  // 一维栅格地图的数据
  std::mutex mutex_;
  int width_{0};
  int height_{0};
  size_t data_size_{0};
  bool is_initialized_{false};

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace minco_local_planner::map_manager

#endif /* __GRID_MAP_H__ */
