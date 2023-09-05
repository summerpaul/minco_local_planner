/**
 * @Author: Yunkai Xia
 * @Date:   2023-09-05 13:21:00
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-09-05 16:55:41
 */
#include <stdint.h>

#ifndef __EDGE_SCORER_H__
#define __EDGE_SCORER_H__
#include <vector>

#include "edge_cost_functions/edge_cost_function.h"
#include "types.h"
namespace minco_local_planner::route {

class EdgeScorer {
 public:
  EdgeScorer() = default;
  ~EdgeScorer() = default;

  bool Init();

  bool Score(const EdgePtr edge, float& score);
  int NumPlugins() const;

 private:
  std::vector<EdgeCostFunction::Ptr> edge_cost_functions_;
};

}  // namespace minco_local_planner::route
#endif /* __EDGE_SCORER_H__ */
