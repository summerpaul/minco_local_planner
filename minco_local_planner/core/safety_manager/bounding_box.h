/**
 * @Author: Xia Yunkai
 * @Date:   2023-08-24 20:01:28
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-08-24 20:15:51
 */
#include <stdint.h>

#ifndef __BOUNDING_BOX_H__
#define __BOUNDING_BOX_H__

#include "basis/data_type.h"

namespace minco_local_planner::runtime_manager {
using namespace basis;
class BoundingBox {
 public:
  explicit BoundingBox(const double& xmin, const double& xmax,
                       const double& ymin, const double& ymax)
      : xmin_(xmin), xmax_(xmax), ymin_(ymin), ymax_(ymax) {}

  bool InBoundingBox(const Vec2d& pt) {
    if (pt[0] < xmin_ || pt[0] > xmax_ || pt[1] < ymin_ || pt[1] > ymax_) {
      return false;
    }
    return true;
  }

  Points2d GetPoints() const {
    Points2d points;
    points.emplace_back(Vec2d(xmin_, ymin_));
    points.emplace_back(Vec2d(xmin_, ymax_));
    points.emplace_back(Vec2d(xmax_, ymax_));
    points.emplace_back(Vec2d(xmax_, ymin_));
    points.emplace_back(Vec2d(xmin_, ymin_));
    return points;
  }

 private:
  double xmin_;
  double xmax_;
  double ymin_;
  double ymax_;
};

}  // namespace minco_local_planner::runtime_manager

#endif /* __BOUNDING_BOX_H__ */
