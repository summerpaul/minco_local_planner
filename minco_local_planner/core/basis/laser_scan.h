/**
 * @Author: Yunkai Xia
 * @Date:   2023-03-22 17:00:14
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-08-24 17:31:36
 */
#include <stdint.h>

#ifndef __LASER_SCAN_H__
#define __LASER_SCAN_H__

#include <vector>
namespace minco_local_planner::basis {

struct LaserScan {
  LaserScan()
      : time_stemp(-1),
        angle_min(0),
        angle_max(0),
        angle_increment(0),
        time_increment(0),
        range_min(0),
        range_max(0) {
    ranges.clear();
    intensities.clear();
  }
  double time_stemp;
  double angle_min;
  double angle_max;
  double angle_increment;
  double time_increment;
  double range_min;
  double range_max;
  std::vector<double> ranges;
  std::vector<double> intensities;
};

}  // namespace minco_local_planner::basis

#endif /* __LASER_SCAN_H__ */
