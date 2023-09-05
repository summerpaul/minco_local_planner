/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-31 08:48:06
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-09-06 00:00:56
 */
#include "plan_manager.h"

#include <iostream>

#include "module_manager/module_manager.h"
// #include "planner_factory/planner_factory.h"
#include "utils/singleton.h"
#include "utils/timer_manager.h"
using namespace std;
namespace minco_local_planner::plan_manager {

using namespace module_manager;
// using namespace planner_factory;
PlanManager::PlanManager() : BaseModule("PlanManager") {}
PlanManager::~PlanManager() {}

bool PlanManager::Init() {
  status_ = PlanStatus::INIT;
  cfg_ =
      ModuleManager::GetInstance()->GetConfigManager()->GetPlanManagerConfig();
  try {
    path_search_plugin_loader_.reset(
        new plugin_loader::PluginLoader(cfg_->path_search_lib_path, false));

  } catch (const std::exception& e) {
    LOG_ERROR("failed to load path search plugin because {} ", e.what());
    return false;
  }
  path_search_ptr_ =
      path_search_plugin_loader_->createSharedInstance<PathSearch>(
          cfg_->path_search_class_name);

  if (!path_search_ptr_) {
    LOG_ERROR("path_search_ptr_ is null ");
    return false;
  }
  if (!path_search_ptr_->Run()) {
    LOG_ERROR("fail to run path_search_ptr_");
    return false;
  }
  path_search_ptr_->SetMap(
      ModuleManager::GetInstance()->GetMapManager()->GetGlobalMap());

  return true;
}
bool PlanManager::Start() {
  using namespace utils;
  Singleton<TimerManager>()->Schedule(
      int(cfg_->plan_sleep_time * 1000),
      std::bind(&PlanManager::ReplanFSMTimer, this));
  return true;
}
void PlanManager::Stop() {
  LOG_INFO("Stop");
  Clear();
  ChangePlanStatus(PlanStatus::INIT);
}

void PlanManager::SetTargetPose(const VehiclePose& pose) {
  LOG_INFO("SetTargetPose");
  target_pose_ = pose;
  ChangePlanStatus(PlanStatus::GLOBAL_PLAN);
}
void PlanManager::Clear() {}

}  // namespace minco_local_planner::plan_manager