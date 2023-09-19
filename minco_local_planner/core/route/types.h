/**
 * @Author: Xia Yunkai
 * @Date:   2023-09-04 21:24:02
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-09-05 11:23:46
 */
#include <stdint.h>

#ifndef __TYPES_H__
#define __TYPES_H__
#include <any>
#include <limits>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "basis/data_type.h"
namespace minco_local_planner::route {
using namespace basis;

struct Metadata {
  Metadata() {}

  template <typename T>
  T GetValue(const std::string& key, T& default_val) const {
    auto it = data.find(key);
    if (it == data.end()) {
      return default_val;
    }
    return std::any_cast<T>(it->second);
  }

  template <typename T>
  void SetValue(const std::string& key, T& value) {
    data[key] = value;
  }

  std::unordered_map<std::string, std::any> data;
};

struct Node;
typedef Node* NodePtr;
typedef std::vector<Node> NodeVector;
typedef NodeVector Graph;
typedef std::unordered_map<unsigned int, unsigned int> GraphToIDMap;
typedef std::vector<NodePtr> NodePtrVector;
typedef std::pair<float, NodePtr> NodeElement;
typedef std::pair<unsigned int, unsigned int> NodeExtents;

// 优先队列的比较器
struct NodeComparator {
  bool operator()(const NodeElement& a, const NodeElement& b) const {
    return a.first > b.first;
  }
};

// 优先队列
typedef std::priority_queue<NodeElement, std::vector<NodeElement>,
                            NodeComparator>
    NodeQueue;

// 图搜索中边的代价
struct EdgeCost {
  float cost{0.0};
  bool overridable{true};  // If overridable, may use plugin edge cost scorers
};

// 路径节点是否启用执行操作
enum class OperationTrigger { NODE = 0, ON_ENTER = 1, ON_EXIT = 2 };

// 执行操作的具体内容
struct Operation {
  std::string type;
  OperationTrigger trigger;
  Metadata metadata;
};

typedef std::vector<Operation> Operations;
typedef std::vector<Operation*> OperationPtrs;

struct OperationsResult {
  std::vector<std::string> operations_triggered;
  bool reroute{false};
  std::vector<unsigned int> blocked_ids;
};

// 双向边存储的内容，更改，使用3阶贝塞尔
struct DirectionalEdge {
  unsigned int edgeid;     // Edge identifier
  NodePtr start{nullptr};  // Ptr to starting node of edge
  NodePtr end{nullptr};    // Ptr to ending node of edge
                           //   Vec2d p1;
                           //   Vec2d p2;
  EdgeCost edge_cost;      // Cost information associated with edge
  Metadata metadata;       // Any metadata stored in the graph file of interest
  Operations operations;   // Operations to perform related to the edge
};

typedef DirectionalEdge* EdgePtr;
typedef std::vector<DirectionalEdge> EdgeVector;
typedef std::vector<EdgePtr> EdgePtrVector;

struct SearchState {
  EdgePtr parent_edge{nullptr};
  float integrated_cost{std::numeric_limits<float>::max()};
  float traversal_cost{std::numeric_limits<float>::max()};

  void Reset() {
    integrated_cost = std::numeric_limits<float>::max();
    traversal_cost = std::numeric_limits<float>::max();
    parent_edge = nullptr;
  }
};

struct Coordinates {
  std::string frame_id{"map"};
  float x{0.0}, y{0.0};
};

struct Node {
  unsigned int nodeid;  // Node identifier
  Coordinates coords;   // Coordinates of node
  EdgeVector neighbors;   // Directed neighbors and edges of the node
  Metadata metadata;      // Any metadata stored in the graph file of interest
  Operations operations;  // Operations to perform related to the node
  SearchState search_state;  // State maintained by route search algorithm
  // 节点增加边
  void AddEdge(EdgeCost& cost, NodePtr node, unsigned int edgeid,
               Metadata meta_data = {}, Operations operations_data = {}) {
    neighbors.push_back({edgeid, this, node, cost, meta_data, operations_data});
  }
};

struct Route {
  NodePtr start_node;
  EdgePtrVector edges;
  float route_cost{0.0};
};

enum class TrackerResult { INTERRUPTED = 0, REROUTE = 1, COMPLETED = 2 };

struct RouteTrackingState {
  NodePtr last_node{nullptr}, next_node{nullptr};
  EdgePtr current_edge{nullptr};
  int route_edges_idx{-1};
  bool within_radius{false};
};

struct ReroutingState {
  unsigned int rerouting_start_id{std::numeric_limits<unsigned int>::max()};
  std::vector<unsigned int> blocked_ids;
  bool first_time{true};
  EdgePtr curr_edge{nullptr};
  Coordinates closest_pt_on_edge;
  Pose2d rerouting_start_pose;

  void Reset() {
    rerouting_start_id = std::numeric_limits<unsigned int>::max();
    blocked_ids.clear();
    first_time = true;
    curr_edge = nullptr;
    closest_pt_on_edge = Coordinates();
    rerouting_start_pose = Pose2d::Zero();
  }
};

}  // namespace minco_local_planner::route

#endif /* __TYPES_H__ */
