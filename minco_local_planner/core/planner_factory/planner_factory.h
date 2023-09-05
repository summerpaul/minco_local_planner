/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-31 10:37:11
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-09-05 16:55:58
 */
#include <stdint.h>

#ifndef __PLANNER_FACTORY_H__
#define __PLANNER_FACTORY_H__
#include "basis/logger.h"
#include "path_search/path_search.h"
#include "route/edge_cost_functions/edge_cost_function.h"
#include "route/graph_file_loaders/graph_file_loader.h"
namespace minco_local_planner::planner_factory {
using namespace path_search;
using namespace route;
class PlannerFactory {
 public:
  static PlannerFactory* GetInstance();

  PathSearch::Ptr CreatGlobalPlanner(int planner_type);

  std::vector<EdgeCostFunction::Ptr> CreateEdgeCostFunctions();
  GraphFileLoader::Ptr CreateGraphFileLoader();

 private:
  PlannerFactory(void) = default;
  ~PlannerFactory(void) = default;
  PlannerFactory(const PlannerFactory& rhs) = delete;
  PlannerFactory& operator=(const PlannerFactory& rhs) = delete;
};
}  // namespace minco_local_planner::planner_factory

#endif /* __PLANNER_FACTORY_H__ */
