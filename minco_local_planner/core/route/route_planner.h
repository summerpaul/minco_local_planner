/**
 * @Author: Yunkai Xia
 * @Date:   2023-09-05 13:05:38
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-09-05 14:15:53
 */
#include <stdint.h>

#ifndef __ROUTE_PLANNER_H__
#define __ROUTE_PLANNER_H__
#include "basis/base_module.h"
#include "basis/logger.h"
#include "edge_scorer.h"
#include "exceptions/route_exceptions.h"
#include "types.h"
namespace minco_local_planner::route {

using namespace basis;
using namespace exceptions;
class RoutePlanner : public BaseModule {
 public:
  RoutePlanner();
  ~RoutePlanner();

  virtual bool Init() override;
  virtual bool Start() override;
  virtual void Stop() override;

  Route FindRoute(Graph& graph, unsigned int start, unsigned int goal,
                  const std::vector<unsigned int>& blocked_ids);

 private:
  inline void ResetSearchStates(Graph& graph);
  // 最短路径搜索
  void FindShortestGraphTraversal(Graph& graph, const NodePtr start,
                                  const NodePtr goal,
                                  const std::vector<unsigned int>& blocked_ids);

  inline bool GetTraversalCost(const EdgePtr edge, float& score,
                               const std::vector<unsigned int>& blocked_ids);
  inline NodeElement GetNextNode();

  inline void AddNode(const float cost, const NodePtr node);

  inline EdgeVector& GetEdges(const NodePtr node);

  inline void ClearQueue();

  inline bool IsGoal(const NodePtr node);

  int max_iterations_{0};
  unsigned int goal_id_{0};
  NodeQueue queue_;
  std::unique_ptr<EdgeScorer> edge_scorer_;
};

}  // namespace minco_local_planner::route

#endif /* __ROUTE_PLANNER_H__ */
