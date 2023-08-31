/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-31 08:48:06
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-08-31 17:13:54
 */
#include "plan_manager.h"

#include <iostream>

#include "module_manager/module_manager.h"
#include "planner_factory/planner_factory.h"
#include "utils/singleton.h"
#include "utils/timer_manager.h"
using namespace std;
namespace minco_local_planner::plan_manager {

using namespace module_manager;
using namespace planner_factory;
PlanManager::PlanManager() : BaseModule("PlanManager") {}
PlanManager::~PlanManager() {}

bool PlanManager::Init() {
  status_ = PlanStatus::INIT;
  cfg_ =
      ModuleManager::GetInstance()->GetConfigManager()->GetPlanManagerConfig();
  path_search_ptr_ =
      PlannerFactory::GetInstance()->CreatGlobalPlanner(cfg_.path_search_type);
  if (!path_search_ptr_) {
    LOG_ERROR("path_search_ptr_ is null ");
    return false;
  }
  if (!path_search_ptr_->Run()) {
    LOG_ERROR("fail to run path_search_ptr_");
    return false;
  }

  return true;
}
bool PlanManager::Start() {
  using namespace utils;
  Singleton<TimerManager>()->Schedule(
      int(100), std::bind(&PlanManager::ReplanFSMTimer, this));
  return true;
}
void PlanManager::Stop() {
  LOG_INFO("Stop");
  Clear();
  ChangePlanStatus(PlanStatus::INIT);
}

void PlanManager::SetTargetPose(const VehiclePose &pose) {
  LOG_INFO("SetTargetPose");
  target_pose_ = pose;
  ChangePlanStatus(PlanStatus::GLOBAL_PLAN);
}
void PlanManager::Clear() {}

}  // namespace minco_local_planner::plan_manager