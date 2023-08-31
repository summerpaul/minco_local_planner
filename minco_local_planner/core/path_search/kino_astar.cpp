/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-30 14:18:36
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-08-31 17:30:19
 */
#include "kino_astar.h"

#include <iostream>
using namespace std;
#include "kino_astar.h"
namespace minco_local_planner::path_search {

KinoAstar::KinoAstar() : PathSearch("KinoAstar") {}


bool KinoAstar::Init() { return true; }
bool KinoAstar::Start() { return true; }
void KinoAstar::Stop() {}

int KinoAstar::Search(const VehiclePose& start_pos, const VehiclePose& end_pos,
                      const Vec2d& init_ctrl) {
  if (!map_ptr_) {
    return NO_MAP;
  }

  
  return NO_PATH;
}

void KinoAstar::Reset() { std::cout << "reset " << std::endl; }
}  // namespace minco_local_planner::path_search
