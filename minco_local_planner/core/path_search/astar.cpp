/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-31 14:32:47
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-08-31 21:49:43
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

bool Astar::Init() {
  using namespace module_manager;
  cfg_ = ModuleManager::GetInstance()->GetConfigManager()->AstarConfig();

  path_node_pool_.resize(cfg_.allocate_num);
      for (int i = 0; i < cfg_.allocate_num; i++) {
      path_node_pool_[i] = std::make_shared<Node>();
    }
  return true;
}
bool Astar::Start() { return true; }
void Astar::Stop() {}
int Astar::Search(const VehiclePose& start_pos, const VehiclePose& end_pos,
                  const Vec2d& init_ctrl) {
  if (!map_ptr_) {
    return NO_MAP;
  }
  Vec2d start_pt = start_pos.GetPos();
  Vec2d end_pt = end_pos.GetPos();
  Vec2i start_pn = map_ptr_->DoubleToInt(start_pt);
  Vec2i end_pn = map_ptr_->DoubleToInt(end_pt);

  if (map_ptr_->IsOccupied(start_pn)) {
    return START_ERR;
  }

  if (map_ptr_->IsOccupied(end_pn)) {
    return END_ERR;
  }

  return NO_PATH;
}

void Astar::Reset() {}

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

// bool Astar::CheckVehiclePose(const VehiclePose& pose) {}

}  // namespace minco_local_planner::path_search