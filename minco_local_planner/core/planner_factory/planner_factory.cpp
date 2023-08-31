/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-31 10:37:15
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-08-31 10:49:22
 */
#include "planner_factory.h"

#include <iostream>

using namespace std;

namespace minco_local_planner::planner_factory {

PlannerFactory* PlannerFactory::GetInstance() {
  static PlannerFactory instance;
  return &instance;
}

PathSearch::Ptr PlannerFactory::CreatGlobalPlanner(int planner_type) {}
}  // namespace minco_local_planner::planner_factory