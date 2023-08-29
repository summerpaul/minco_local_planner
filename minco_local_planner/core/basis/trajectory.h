/**
 * @Author: Xia Yunkai
 * @Date:   2023-08-24 20:20:58
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-08-29 20:56:08
 */
#include <stdint.h>

#ifndef __TRAJECTORY_H__
#define __TRAJECTORY_H__
#include "math.h"
#include "trajectory_point.h"

namespace minco_local_planner::basis {

double Cross2(const Vec2d &a, const Vec2d &b) {
  return a[0] * b[1] - a[1] * b[0];
}

double LineDistance(const Vec2d &a, const Vec2d &b, const Vec2d &c) {
  return Cross2((b - a), (c - a)) / (b - a).norm();
}

double LineStripDistanceSigned(const Vec2d &a, const Vec2d &b, const Vec2d &c) {
  if ((b - a).dot(c - a) <= 0) return -(c - a).norm();
  if ((a - b).dot(c - b) <= 0) return (c - b).norm();
  return std::abs(LineDistance(a, b, c));
}

class Trajectory : public vec_E<TrajectoryPoint> {
 public:
  using Iterator = vec_E<TrajectoryPoint>::iterator;
  using ConstIterator = vec_E<TrajectoryPoint>::const_iterator;

 public:
  double Length() const {
    double l = 0;
    for (size_t i = 1; i < size(); i++) {
      l += ((*this)[i - 1].GetPos() - (*this)[i].GetPos()).norm();
    }
    return l;
  }

  ConstIterator FindNearest(const ConstIterator &begin,
                            const ConstIterator &end, const Vec2d &target,
                            const double max_search_range = 0,
                            const double &distance = 0) const {
    if (begin == end) {
      return end;
    }
    double distance_path_search = 0;
    ConstIterator it_nearest = begin;
    double min_dist = (begin->GetPos() - target).norm();
    ConstIterator it_prev = begin;
    for (ConstIterator it = begin + 1; it != end; ++it) {
      const Vec2d inc = it->GetPos() - it_prev->GetPos();
      distance_path_search += inc.norm();
      if (max_search_range > 0 && distance_path_search > max_search_range) {
        break;
      }
      const double d =
          LineStripDistanceSigned(it_prev->GetPos(), it->GetPos(), target);
      const double d_compare = (d > 0) ? d : (-d - EPSION);
      const double d_abs = std::abs(d);
      if (d_compare <= min_dist ||
          (it + 1 == end && d_abs <= min_dist + EPSION)) {
        min_dist = d_abs;
        it_nearest = it;
      }
      it_prev = it;
    }
    if (distance == 0) {
      return it_nearest;
    }

    for (ConstIterator it = it_nearest + 1; it < end; ++it) {
      const double dis = (target - it->GetPos()).norm();
      if (dis > distance) {
        it_nearest = it;
        break;
      }
    }

    return it_nearest;
  }

  void EmplaceBack(TrajectoryPoint pt) {
    if (this->size() == 0) {
      pt.SetS(0);
      this->emplace_back(pt);
      return;
    }

    TrajectoryPoint back_point = this->back();
    const double last_s = back_point.GetS();
    const Vec2d diff = pt.GetPos() - back_point.GetPos();
    const double add_s = diff.norm();
    pt.SetS(last_s + add_s);
    this->emplace_back(pt);
    return;
  }

  void TrapezoidSpeedReplan(const double &start_vel, const double &target_vel,
                            const double &end_vel, const double &acc,
                            const double &dec, const double &creep_dist,
                            const double &creep_speed,
                            const double &start_creep_dist) {
    double sa = 0, sc = 0, sd = 0, se = 0, s0 = 0;
    double target_speed = target_vel;
    double init_speed = start_vel;
    double distance = Length();
    double end_speed = end_vel;

    if (init_speed < 0.05) {
      s0 = start_creep_dist;
      init_speed = 0.05;
      if (target_speed < init_speed) {
        init_speed = target_vel;
      }
    }

    if (end_speed <= creep_speed) {
      end_speed = creep_speed;
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
    double speed = 0;
    for (auto it = this->begin(); it != this->end(); ++it) {
      const double dist = it->GetS();
      if (dist < s0) {
        speed = init_speed;
      }

      else if ((dist >= s0) && (dist <= sa + s0)) {
        speed = sqrt(2 * dist * acc + init_speed * init_speed);

        if (target_speed < init_speed) {
          double speed2 = init_speed * init_speed - 2 * dist * acc;
          if (speed2 < 0)
            speed = target_speed;
          else
            speed = sqrt(speed2);
        }
      } else if ((dist > sa + s0) && (dist <= (sa + sc + s0))) {
        speed = target_speed;

      }

      else if ((dist > (sa + sc + s0)) && (dist <= (sa + sc + sd + s0))) {
        double bb = dist - sa - sc - s0;
        double speed2 = -2 * bb * dec + target_speed * target_speed;
        if (speed2 < 0)
          speed = end_speed;
        else
          speed = sqrt(speed2);

        if (target_speed < end_speed) {
          speed = sqrt(2 * bb * dec + target_speed * target_speed);
        }
      } else if ((dist > (sa + sc + sd + s0)) &&
                 (dist <= (sa + sc + sd + se + s0))) {
        speed = end_speed;
      }

      it->SetVel(speed);
    }
  }
};

}  // namespace minco_local_planner::basis

#endif /* __TRAJECTORY_H__ */
