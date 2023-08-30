/**
 * @Author: Xia Yunkai
 * @Date:   2023-08-24 20:01:28
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-08-29 19:42:07
 */
#include <stdint.h>

#ifndef __BOUNDING_BOX_H__
#define __BOUNDING_BOX_H__
#include <string>

#include "basis/data_type.h"

namespace minco_local_planner::safety_manager {
using namespace basis;

class BoundingBox {
 public:
  BoundingBox() {
    xmin_ = std::numeric_limits<double>::max();
    xmax_ = std::numeric_limits<double>::min();
    ymin_ = std::numeric_limits<double>::max();
    ymax_ = std::numeric_limits<double>::min();
    name_ = "none";
  }
  BoundingBox(const double& xmin, const double& xmax, const double& ymin,
              const double& ymax, const std::string& name)
      : xmin_(xmin), xmax_(xmax), ymin_(ymin), ymax_(ymax), name_(name) {}
  BoundingBox(const double& xmin, const double& xmax, const double& ymin,
              const double& ymax, const double& x_margin,
              const double& y_margin, const std::string& name)
      : xmin_(xmin - x_margin),
        xmax_(xmax + x_margin),
        ymin_(ymin - y_margin),
        ymax_(ymax + y_margin),
        name_(name) {}
  bool InBoundingBox(const Vec2d& pt) {
    if (pt[0] < xmin_ || pt[0] > xmax_ || pt[1] < ymin_ || pt[1] > ymax_) {
      return false;
    }
    return true;
  }

  bool InBoundingBox(const double& x, const double& y) {
    if (x < xmin_ || x > xmax_ || y < ymin_ || y > ymax_) {
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
  const std::string Name() const { return name_; }

 private:
  double xmin_;
  double xmax_;
  double ymin_;
  double ymax_;
  std::string name_;
};

}  // namespace minco_local_planner::safety_manager

#endif /* __BOUNDING_BOX_H__ */
