/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-30 15:11:13
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-08-30 15:41:43
 */
#include <stdint.h>

#ifndef __RAYCAST_H__
#define __RAYCAST_H__

#include "basis/data_type.h"
#include "basis/math.h"
namespace minco_local_planner::safety_manager {
using namespace basis;

class RayCaster {
 public:
  RayCaster() {}
  ~RayCaster() {}
  bool SetInput(const Vec2d& start, const Vec2d& end) {
    start_ = start;
    end_ = end;
    x_ = (int)std::floor(start_.x());
    y_ = (int)std::floor(start_.y());
    endX_ = (int)std::floor(end_.x());
    endY_ = (int)std::floor(end_.y());
    direction_ = (end_ - start_);
    maxDist_ = direction_.squaredNorm();
    dx_ = endX_ - x_;
    dy_ = endY_ - y_;
    stepX_ = (int)Sign((int)dx_);
    stepY_ = (int)Sign((int)dy_);
    tMaxX_ = Intbound(start_.x(), dx_);
    tMaxY_ = Intbound(start_.y(), dy_);
    tDeltaX_ = ((double)stepX_) / dx_;
    tDeltaY_ = ((double)stepY_) / dy_;
    dist_ = 0;

    step_num_ = 0;
    if (stepX_ == 0 && stepY_ == 0)
      return false;
    else
      return true;
  }
  bool Step(Vec2d& ray_pt) {
    ray_pt = Vec2d(x_, y_);
    if (x_ == endX_ && y_ == endY_) {
      return false;
    }

    if (tMaxX_ < tMaxY_) {
      x_ += stepX_;

      tMaxX_ += tDeltaX_;

    } else {
      y_ += stepY_;
      tMaxY_ += tDeltaY_;
    }

    return true;
  }

 private:
  Vec2d start_, end_;
  Vec2d direction_;
  Vec2d min_, max_;
  int x_, y_;
  int endX_, endY_;
  double maxDist_;
  double dx_, dy_;
  int stepX_, stepY_;
  double tMaxX_, tMaxY_;
  double tDeltaX_, tDeltaY_;
  double dist_;
  int step_num_;
};
}  // namespace minco_local_planner::safety_manager
#endif /* __RAYCAST_H__ */
