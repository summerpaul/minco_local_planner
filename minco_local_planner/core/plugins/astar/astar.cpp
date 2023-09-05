/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-31 14:32:47
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-09-05 22:12:00
 */
#include <iostream>

using namespace std;
#include "astar.h"
#include "module_manager/module_manager.h"
namespace minco_local_planner::path_search {

Astar::Astar() : PathSearch("Astar") {
  motions_.emplace_back(Vec2i(-1, -1));
  motions_.emplace_back(Vec2i(-1, 0));
  motions_.emplace_back(Vec2i(-1, 1));
  motions_.emplace_back(Vec2i(0, -1));
  motions_.emplace_back(Vec2i(0, 1));
  motions_.emplace_back(Vec2i(1, -1));
  motions_.emplace_back(Vec2i(1, 0));
  motions_.emplace_back(Vec2i(1, 1));
}
Astar::~Astar() {
  for (int i = 0; i < cfg_->allocate_num; i++) {
    delete path_node_pool_[i];
  }
}
bool Astar::Init() {
  using namespace module_manager;
  cfg_ = ModuleManager::GetInstance()->GetConfigManager()->GetAstarConfig();
  path_node_pool_.resize(cfg_->allocate_num);

  for (int i = 0; i < cfg_->allocate_num; i++) {
    path_node_pool_[i] = new PathNode;
  }
  use_node_num_ = 0;
  iter_num_ = 0;
  return true;
}

bool Astar::Start() { return true; }

void Astar::Stop() {}

int Astar::Search(const VehiclePose& start_pos, const VehiclePose& end_pos,
                  const Vec2d& init_ctrl) {
  if (!map_ptr_) {
    return NO_MAP;
  }
  Pose2d start_pt = start_pos.GetPose2d();
  Pose2d end_pt = end_pos.GetPose2d();
  Vec2i start_pn = map_ptr_->DoubleToInt(start_pt.head(2));
  Vec2i end_pn = map_ptr_->DoubleToInt(end_pt.head(2));

  if (map_ptr_->IsOccupied(start_pn)) {
    return START_ERR;
  }

  if (map_ptr_->IsOccupied(end_pn)) {
    return END_ERR;
  }
  end_pt_ = end_pt.head(2);
  PathNodePtr cur_node = path_node_pool_[0];
  cur_node->pose = start_pt;
  cur_node->parent = nullptr;
  cur_node->index = start_pn;
  cur_node->g_score = 0;
  cur_node->f_score = cfg_->lambda_heu * GetHeu(Vec2d(start_pn[0], start_pn[1]),
                                                Vec2d(end_pn[0], end_pn[1]),
                                                cfg_->time_breaker);
  cur_node->node_state = IN_OPEN_SET;
  open_set_.push(cur_node);
  use_node_num_ += 1;
  PathNodePtr neighbor{nullptr}, terminate_node{nullptr};
  int iter_num = 0;
  while (!open_set_.empty()) {
    iter_num++;
    cur_node = open_set_.top();
    bool reach_end = abs(cur_node->index(0) - end_pn(0)) <= 1 &&
                     abs(cur_node->index(1) - end_pn(1)) <= 1;
    if (reach_end) {
      terminate_node = cur_node;
      LOG_TRACE("reach end, use_node_num is {}, iter_num is {}", use_node_num_,
                iter_num);
      RetrievePath(terminate_node);
      return REACH_END;
    }
    open_set_.pop();
    cur_node->node_state = IN_CLOSE_SET;
    for (auto& motion : motions_) {
      Vec2i neighbor_pn = cur_node->index + motion;
      if (!map_ptr_->Query(neighbor_pn)) {
        continue;
      }
      neighbor = expanded_nodes_.find(neighbor_pn);
      if (neighbor != nullptr && neighbor->node_state == IN_CLOSE_SET) continue;
      double tmp_g = Vec2d(motion[0], motion[1]).norm() + cur_node->g_score;
      double tmp_f =
          tmp_g + cfg_->lambda_heu *
                      GetHeu(Vec2d(neighbor_pn[0], neighbor_pn[1]),
                             Vec2d(end_pn[0], end_pn[1]), cfg_->time_breaker);

      if (neighbor == nullptr) {
        neighbor = path_node_pool_[use_node_num_];
        neighbor->index = neighbor_pn;
        neighbor->pose.head(2) = map_ptr_->IntToDouble(neighbor_pn);
        neighbor->f_score = tmp_f;
        neighbor->g_score = tmp_g;
        neighbor->parent = cur_node;
        neighbor->node_state = IN_OPEN_SET;
        open_set_.push(neighbor);
        expanded_nodes_.insert(neighbor_pn, neighbor);
        use_node_num_ += 1;
        if (use_node_num_ == cfg_->allocate_num) {
          LOG_WARN("run out of memory.");
          return NO_PATH;
        }
      } else if (neighbor->node_state == IN_OPEN_SET) {
        neighbor->parent = cur_node;
        neighbor->f_score = tmp_f;
        neighbor->g_score = tmp_g;
      } else {
        LOG_WARN("error type in searching");
      }
    }
  }

  return NO_PATH;
}

void Astar::RetrievePath(PathNodePtr end_node) {
  PathNodePtr cur_node = end_node;
  path_nodes_.emplace_back(cur_node);
  while (cur_node->parent != NULL) {
    cur_node = cur_node->parent;
    path_nodes_.emplace_back(cur_node);
  }

  reverse(path_nodes_.begin(), path_nodes_.end());
}

void Astar::Reset() {
  expanded_nodes_.clear();
  path_nodes_.clear();

  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator>
      empty_queue;
  open_set_.swap(empty_queue);
  for (int i = 0; i < use_node_num_; i++) {
    PathNodePtr node = path_node_pool_[i];
    node->parent = NULL;
    node->node_state = NOT_EXPAND;
  }

  use_node_num_ = 0;
  iter_num_ = 0;
}

double Astar::GetHeu(const Vec2d& x1, const Vec2d& x2,
                     const double& tie_breaker) {
  if (cfg_->heu_type == 0) {
    return GetEuclHeu(x1, x2, tie_breaker);
  }

  else if (cfg_->heu_type == 1) {
    return GetManhHeu(x1, x2, tie_breaker);
  } else {
    return GetDiagHeu(x1, x2, tie_breaker);
  }
}

double Astar::GetEuclHeu(const Vec2d& x1, const Vec2d& x2,
                         const double& tie_breaker) {
  return tie_breaker * (x2 - x1).norm();
}

double Astar::GetDiagHeu(const Vec2d& x1, const Vec2d& x2,
                         const double& tie_breaker) {
  double dx = fabs(x1(0) - x2(0));
  double dy = fabs(x1(1) - x2(1));
  double h = (dx + dy) + (sqrt(2) - 2) * std::min(dx, dy);
  return tie_breaker * h;
}

double Astar::GetManhHeu(const Vec2d& x1, const Vec2d& x2,
                         const double& tie_breaker) {
  double dx = fabs(x1(0) - x2(0));
  double dy = fabs(x1(1) - x2(1));
  return tie_breaker * (dx + dy);
}
void Astar::GetPath2D(Path2d& path) {
  path.clear();
  for (size_t i = 0; i < path_nodes_.size(); ++i) {
    path.emplace_back(path_nodes_[i]->pose.head(2));
  }
  path.emplace_back(end_pt_);
}
// bool Astar::CheckVehiclePose(const VehiclePose& pose) {}

}  // namespace minco_local_planner::path_search

#include "plugin_loader/plugin_loader.hpp"
PLUGIN_LOADER_REGISTER_CLASS(minco_local_planner::path_search::Astar,
                             minco_local_planner::path_search::PathSearch)