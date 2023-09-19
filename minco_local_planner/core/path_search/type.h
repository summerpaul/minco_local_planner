/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-31 08:55:34
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-08-31 17:25:09
 */
#include <stdint.h>

#ifndef __TYPE_H__
#define __TYPE_H__
#include <queue>
#include <unordered_map>

#include "basis/base_module.h"
#include "basis/data_type.h"
#include "basis/logger.h"
namespace minco_local_planner::path_search {
using namespace basis;
constexpr char IN_CLOSE_SET = 'a';
constexpr char IN_OPEN_SET = 'b';
constexpr char NOT_EXPAND = 'c';
constexpr double inf = 1 >> 30;
enum { REACH_END = 1, NO_PATH, NO_MAP, START_ERR, END_ERR };

// 搜索的节点
struct PathNode {
  Vec2i index;
  int yaw_idx;
  Pose2d pose;
  double g_score, f_score;
  Vec3d input;  // vx. vy, omega
  PathNode* parent;
  char node_state;
  int singul = 0;
  int number;
  PathNode() {
    parent = nullptr;
    node_state = NOT_EXPAND;
  }
  ~PathNode(){};
};

typedef PathNode* PathNodePtr;

struct MiddleNode {
  double start_s;
  double start_t;
  double start_vel;
  double end_s;
  double end_t;
  double end_vel;
  Vec3i s_t_v_idx;
  double length;
  double distance2end;
  double acc;
  double g_score, f_score;
  char node_state;

  MiddleNode* parent;

  MiddleNode() {
    parent = NULL;
    node_state = NOT_EXPAND;
  }
  ~MiddleNode(){};
};
typedef MiddleNode* MiddleNodePtr;

class NodeComparator {
 public:
  template <class NodePtr>
  bool operator()(NodePtr node1, NodePtr node2) {
    return node1->f_score > node2->f_score;
  }
};

template <typename T>
struct matrix_hash : std::unary_function<T, size_t> {
  std::size_t operator()(T const& matrix) const {
    size_t seed = 0;
    for (long int i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) +
              (seed >> 2);
    }
    return seed;
  }
};

template <class NodePtr>
class NodeHashTable {
 private:
  std::unordered_map<Vec2i, NodePtr, matrix_hash<Vec2i>> data_2d_;
  std::unordered_map<Vec3i, NodePtr, matrix_hash<Vec3i>> data_3d_;

 public:
  NodeHashTable(/* args */) {}
  ~NodeHashTable() {}
  // : for 2d vehicle planning
  void insert(Vec2i idx, NodePtr node) {
    data_2d_.insert(std::make_pair(idx, node));
  }
  // for 3d vehicle planning
  void insert(Vec2i idx, int yaw_idx, NodePtr node) {
    data_3d_.insert(std::make_pair(Vec3i(idx(0), idx(1), yaw_idx), node));
  }
  void insert(Vec3i idx, NodePtr node) {
    data_3d_.insert(std::make_pair(idx, node));
  }

  NodePtr find(Vec2i idx) {
    auto iter = data_2d_.find(idx);
    return iter == data_2d_.end() ? NULL : iter->second;
  }
  NodePtr find(Vec2i idx, int yaw_idx) {
    auto iter = data_3d_.find(Vec3i(idx(0), idx(1), yaw_idx));
    return iter == data_3d_.end() ? NULL : iter->second;
  }

  void clear() {
    data_2d_.clear();
    data_3d_.clear();
  }
};

template <class MiddleNode>
class MiddleNodeHashTable {
 private:
  /* data */
  std::unordered_map<Vec3i, MiddleNodePtr, matrix_hash<Vec3i>> middle_data_3d_;

 public:
  MiddleNodeHashTable(/* args */) {}
  ~MiddleNodeHashTable() {}

  // For middle node searching
  void insert(Vec3i idx, MiddleNodePtr node) {
    middle_data_3d_.insert(std::make_pair(idx, node));
  }

  MiddleNodePtr find(Vec3i idx) {
    auto iter = middle_data_3d_.find(idx);
    return iter == middle_data_3d_.end() ? NULL : iter->second;
  }

  void clear() { middle_data_3d_.clear(); }
};
}  // namespace minco_local_planner::path_search

#endif /* __TYPE_H__ */
