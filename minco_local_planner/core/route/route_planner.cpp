/**
 * @Author: Yunkai Xia
 * @Date:   2023-09-05 13:05:48
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-09-05 14:28:19
 */
#include "route_planner.h"

#include <iostream>
using namespace std;

namespace minco_local_planner::route {

RoutePlanner::RoutePlanner() : BaseModule("RoutePlanner") {}
RoutePlanner::~RoutePlanner() {}

bool RoutePlanner::Init() { return true; }
bool RoutePlanner::Start() { return true; }
void RoutePlanner::Stop() {}

Route RoutePlanner::FindRoute(Graph& graph, unsigned int start,
                              unsigned int goal,
                              const std::vector<unsigned int>& blocked_ids) {
  if (graph.empty()) {
    LOG_ERROR("Graph is invalid for routing!");
    throw NoValidGraph("Graph is invalid for routing!");
  }
  const NodePtr& start_node = &graph.at(start);
  const NodePtr& goal_node = &graph.at(goal);
  FindShortestGraphTraversal(graph, start_node, goal_node, blocked_ids);
  EdgePtr& parent_edge = goal_node->search_state.parent_edge;
  if (!parent_edge) {
    LOG_ERROR("Could not find a route to the requested goal!");
    throw NoValidRouteCouldBeFound(
        "Could not find a route to the requested goal!");
  }
  Route route;
  while (parent_edge) {
    route.edges.push_back(parent_edge);
    parent_edge = parent_edge->start->search_state.parent_edge;
  }

  std::reverse(route.edges.begin(), route.edges.end());
  route.start_node = start_node;
  route.route_cost = goal_node->search_state.integrated_cost;
  return route;
}

void RoutePlanner::ResetSearchStates(Graph& graph) {
  for (unsigned int i = 0; i != graph.size(); i++) {
    graph[i].search_state.Reset();
  }
}

void RoutePlanner::FindShortestGraphTraversal(
    Graph& graph, const NodePtr start, const NodePtr goal,
    const std::vector<unsigned int>& blocked_ids) {
  ResetSearchStates(graph);
  goal_id_ = goal->nodeid;
  start->search_state.integrated_cost = 0.0;
  AddNode(0.0, start);
  NodePtr neighbor{nullptr};
  EdgePtr edge{nullptr};
  float potential_cost = 0.0, traversal_cost = 0.0;
  int iterations = 0;
  while (!queue_.empty() && iterations < max_iterations_) {
    iterations++;

    auto [curr_cost, node] = GetNextNode();
    if (curr_cost != node->search_state.integrated_cost) {
      continue;
    }
    if (IsGoal(node)) {
      break;
    }

    EdgeVector& edges = GetEdges(node);

    for (unsigned int edge_num = 0; edge_num != edges.size(); edge_num++) {
      edge = &edges[edge_num];
      neighbor = edge->end;

      if (!GetTraversalCost(edge, traversal_cost, blocked_ids)) {
        continue;
      }
      potential_cost = curr_cost + traversal_cost;
      if (potential_cost < neighbor->search_state.integrated_cost) {
        neighbor->search_state.parent_edge = edge;
        neighbor->search_state.integrated_cost = potential_cost;
        neighbor->search_state.traversal_cost = traversal_cost;
        AddNode(potential_cost, neighbor);
      }
    }
  }

  ClearQueue();
    if (iterations >= max_iterations_) {
    throw TimedOut("Maximum iterations was exceeded!");
  }
}

void RoutePlanner::AddNode(const float cost, const NodePtr node) {
  queue_.emplace(cost, node);
}

NodeElement RoutePlanner::GetNextNode() {
  NodeElement data = queue_.top();
  queue_.pop();
  return data;
}

bool RoutePlanner::IsGoal(const NodePtr node) {
  return node->nodeid == goal_id_;
}

EdgeVector& RoutePlanner::GetEdges(const NodePtr node) {
  return node->neighbors;
}

bool RoutePlanner::GetTraversalCost(
    const EdgePtr edge, float& score,
    const std::vector<unsigned int>& blocked_ids) {
  auto idBlocked = [&](unsigned int id) {
    return id == edge->edgeid || id == edge->end->nodeid;
  };
  auto is_blocked =
      std::find_if(blocked_ids.begin(), blocked_ids.end(), idBlocked);
  if (is_blocked != blocked_ids.end() && !IsGoal(edge->end)) {
    return false;
  }

  if (!edge->edge_cost.overridable || edge_scorer_->NumPlugins() == 0) {
    if (edge->edge_cost.cost == 0.0) {
      throw NoValidGraph(
          "Edge " + std::to_string(edge->edgeid) +
          " doesn't contain and cannot compute a valid edge cost!");
    }
    score = edge->edge_cost.cost;
    return true;
  }

  return edge_scorer_->Score(edge, score);
}

void RoutePlanner::ClearQueue() {
  NodeQueue q;
  std::swap(queue_, q);
}

}  // namespace minco_local_planner::route
