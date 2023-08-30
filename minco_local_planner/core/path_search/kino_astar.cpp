/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-30 14:18:36
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-08-30 15:34:21
 */
#include "kino_astar.h"

#include <iostream>
using namespace std;
#include "kino_astar.h"
namespace minco_local_planner::path_search {

KinoAstar::KinoAstar() : PathSearch("KinoAstar") {}

int KinoAstar::Search(const VehiclePose& start_pos, const Vec2d& init_ctrl,
                      const VehiclePose& end_pos) {}

void KinoAstar::Reset() { std::cout << "reset " << std::endl; }
}  // namespace minco_local_planner::path_search
