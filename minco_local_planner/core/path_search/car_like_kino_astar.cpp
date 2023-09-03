/**
 * @Author: Xia Yunkai
 * @Date:   2023-09-02 16:54:25
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-09-02 22:28:43
 */
#include <iostream>

using namespace std;

#include "car_like_kino_astar.h"
#include "module_manager/module_manager.h"
namespace minco_local_planner::path_search {

CarLikeKinoAstar::CarLikeKinoAstar() : PathSearch("CarLikeKinoAstar") {}

CarLikeKinoAstar::~CarLikeKinoAstar() {
  for (int i = 0; i < cfg_->allocate_num; i++) {
    delete path_node_pool_[i];
  }
}
int CarLikeKinoAstar::Search(const VehiclePose& start_pos,
                             const VehiclePose& end_pos,
                             const Vec2d& init_ctrl) {}

void CarLikeKinoAstar::Reset() {}
bool CarLikeKinoAstar::Init() {
  using namespace module_manager;
  cfg_ = ModuleManager::GetInstance()
             ->GetConfigManager()
             ->GetCarLikeKinoAstarConfig();
  path_node_pool_.resize(cfg_->allocate_num);

  for (int i = 0; i < cfg_->allocate_num; i++) {
    path_node_pool_[i] = new PathNode;
    middle_node_pool_[i] = new MiddleNode;
  }
  path_nodes_.clear();
  middle_nodes_.clear();
  last_path_pos_.clear();
  last_path_pos_temp_.clear();

  use_node_num_ = 0;
  use_time_node_num_ = 0;
  iter_num_ = 0;
  return true;
}
bool CarLikeKinoAstar::Start() {}
void CarLikeKinoAstar::Stop() {}
void CarLikeKinoAstar::GetPath2D(Path2d& path) {}
}  // namespace minco_local_planner::path_search