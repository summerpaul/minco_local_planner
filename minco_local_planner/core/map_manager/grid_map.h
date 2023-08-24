/**
 * @Author: Xia Yunkai
 * @Date:   2023-08-24 21:20:40
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-08-24 23:01:52
 */
#include <stdint.h>

#ifndef __GRID_MAP_H__
#define __GRID_MAP_H__
#include <omp.h>

#include <memory>
#include <mutex>

#include "basis/data_type.h"
#include "basis/rigid2d.h"
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
  virtual ~GridMap() = default;

  GridMap(const Pose2d &origin, const Vec2i &dim,
          const std::vector<int8_t> &data, const double &res) {
    CreateGridMap(origin, dim, data, res);
  }

  void CreateGridMap(const Pose2d &origin, const Vec2i &dim,
                     const std::vector<int8_t> &data, const double &res) {
    data_mutex_.lock();
    res_ = res;
    res_inv_ = 1 / res;
    origin_ = origin;
    dim_ = dim;
    data_ = data;
    trans_ = Transform2D(origin_);
    trans_inv_ = trans_.Inv();
    data_mutex_.unlock();
  }

  const double GetRes() const { return res_; }
  const Pose2d GetOrigin() const { return origin_; }
  const Vec2i GetDim() const { return dim_; }
  const std::vector<int8_t> &GetData() const { return data_; }

  Vec2i DoubleToInt(const Vec2d &pt) {
    // 全局坐标系转局部坐标系
    Vec2d point = trans_inv_(pt);
    Vec2i pn;
    pn[0] = std::round((point[0]) / res_ - 0.5);
    pn[1] = std::round((point[1]) / res_ - 0.5);

    return pn;
  }
  Vec2d IntToDouble(const Vec2i &pn) {
    // 局部坐标系转全局坐标系
    return trans_((pn.template cast<double>() + Vec2d::Constant(0.5)) * res_);
  }

 protected:
  Pose2d origin_;
  Transform2D trans_, trans_inv_;
  Vec2i dim_;
  double res_;                // 栅格地图的分辨率
  double res_inv_;            // 栅格地图分辨率的倒数
  std::vector<int8_t> data_;  // 一维栅格地图的数据
  std::mutex data_mutex_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace minco_local_planner::map_manager

#endif /* __GRID_MAP_H__ */
