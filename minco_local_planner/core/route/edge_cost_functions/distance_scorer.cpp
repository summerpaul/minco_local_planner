/**
 * @Author: Yunkai Xia
 * @Date:   2023-09-05 13:44:10
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-09-05 13:56:24
 */
#include "distance_scorer.h"

#include <iostream>

namespace minco_local_planner::route {

bool DistanceScorer::Init() { name_ = "DistanceScorer"; }

bool DistanceScorer::Score(const EdgePtr edge, float& cost) {
  float speed_val = 1.0f;
  speed_val = edge->metadata.GetValue<float>(speed_tag_, speed_val);
  cost = weight_ *
         hypotf(edge->end->coords.x - edge->start->coords.x,
                edge->end->coords.y - edge->start->coords.y) /
         speed_val;
  return true;
}

std::string DistanceScorer::GetName() { return name_; }

}  // namespace minco_local_planner::route