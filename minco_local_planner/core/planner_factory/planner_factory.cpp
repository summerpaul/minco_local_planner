/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-31 10:37:15
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-08-31 17:12:40
 */
#include "planner_factory.h"

#include <iostream>

#include "basis/planner_const.h"
#include "path_search/astar.h"
#include "path_search/kino_astar.h"
using namespace std;

namespace minco_local_planner::planner_factory {
using namespace basis;
using namespace path_search;
PlannerFactory* PlannerFactory::GetInstance() {
  static PlannerFactory instance;
  return &instance;
}

PathSearch::Ptr PlannerFactory::CreatGlobalPlanner(int planner_type) {
  PathSearch::Ptr path_search_ptr_;
  if (planner_type == int(PathSearchType::ASTAR)) {
    path_search_ptr_.reset(new Astar);
    LOG_INFO("create Astar");
  } else if (planner_type == int(PathSearchType::KINO_ASTAR)) {
    path_search_ptr_.reset(new KinoAstar);
    LOG_INFO("create KinoAstar");
  } else {
    path_search_ptr_ = nullptr;
  }
  return path_search_ptr_;
}
}  // namespace minco_local_planner::planner_factory