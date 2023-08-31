/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-31 14:32:47
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-08-31 17:30:11
 */
#include <iostream>

using namespace std;
#include "astar.h"
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

bool Astar::Init() { return true; }
bool Astar::Start() { return true; }
void Astar::Stop() {}
int Astar::Search(const VehiclePose& start_pos, const VehiclePose& end_pos,
                  const Vec2d& init_ctrl) {
  if (!map_ptr_) {
    return NO_MAP;
  }

  if (map_ptr_->IsOccupied(start_pos.GetPos())) {
    return START_ERR;
  }

  return NO_PATH;
}

void Astar::Reset() {}

bool Astar::CheckVehiclePose(const VehiclePose& pose) {}

}  // namespace minco_local_planner::path_search