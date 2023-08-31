/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-31 10:37:11
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-08-31 10:48:35
 */
#include <stdint.h>

#ifndef __PLANNER_FACTORY_H__
#define __PLANNER_FACTORY_H__
#include "basis/logger.h"
#include "path_search/path_search.h"
namespace minco_local_planner::planner_factory {
using namespace path_search;
class PlannerFactory {
 public:
  static PlannerFactory* GetInstance();

  PathSearch::Ptr CreatGlobalPlanner(int planner_type);

 private:
  PlannerFactory(void) = default;
  ~PlannerFactory(void) = default;
  PlannerFactory(const PlannerFactory& rhs) = delete;
  PlannerFactory& operator=(const PlannerFactory& rhs) = delete;
};
}  // namespace minco_local_planner::planner_factory

#endif /* __PLANNER_FACTORY_H__ */
