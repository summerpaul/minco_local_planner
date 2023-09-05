/**
 * @Author: Yunkai Xia
 * @Date:   2023-09-05 13:34:54
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-09-05 13:52:19
 */
#include <stdint.h>

#ifndef __DISTANCE_SCORER_H__
#define __DISTANCE_SCORER_H__

#include "edge_cost_function.h"

namespace minco_local_planner::route {

class DistanceScorer : public EdgeCostFunction {
 public:
  DistanceScorer() = default;
  virtual ~DistanceScorer() = default;

  virtual bool Init() = 0;

  virtual bool Score(const EdgePtr edge, float& cost) override;

  virtual std::string GetName() override;


 protected:
  std::string name_;
  std::string speed_tag_;
  float weight_;
};
}  // namespace minco_local_planner::route
#endif /* __DISTANCE_SCORER_H__ */
