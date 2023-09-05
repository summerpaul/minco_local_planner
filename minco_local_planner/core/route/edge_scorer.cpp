/**
 * @Author: Yunkai Xia
 * @Date:   2023-09-05 13:21:59
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-09-05 13:58:27
 */
#include "edge_scorer.h"

#include <iostream>
using namespace std;

namespace minco_local_planner::route {

bool EdgeScorer::Init() {
  // 从配置文件获取边界代价类型
}
bool EdgeScorer::Score(const EdgePtr edge, float& total_score) {
  total_score = 0.0;
  float curr_score = 0.0;

  for (auto& plugin : edge_cost_functions_) {
    plugin->Prepare();
  }
  for (auto& plugin : edge_cost_functions_) {
    curr_score = 0.0;
    if (plugin->Score(edge, curr_score)) {
      total_score += curr_score;
    } else {
      return false;
    }
  }

  return true;
}
int EdgeScorer::NumPlugins() const { return edge_cost_functions_.size(); }
}  // namespace minco_local_planner::route