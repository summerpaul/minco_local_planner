/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-31 08:48:06
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-08-31 10:25:46
 */
#include "plan_manager.h"

#include <iostream>

#include "utils/singleton.h"
#include "utils/timer_manager.h"
using namespace std;
namespace minco_local_planner::plan_manager {
PlanManager::PlanManager() : BaseModule("PlanManager") {}
PlanManager::~PlanManager() {}

bool PlanManager::Init() {
  status_ = PlanStatus::INIT;
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