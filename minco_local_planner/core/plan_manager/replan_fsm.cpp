/**
 * @Author: Xia Yunkai
 * @Date:   2023-08-24 19:58:32
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-09-01 23:47:41
 */
#include <iostream>

#include "module_manager/module_manager.h"
#include "plan_manager.h"
namespace minco_local_planner::plan_manager {
using namespace module_manager;
using namespace runtime_manager;
void PlanManager::ReplanFSMTimer() {
  //   std::cout << "int ReplanFSMTimer " << std::endl;
  static int log_count = 0;

  const auto runtime_status =
      ModuleManager::GetInstance()->GetRuntimeManager()->GetStatus();

  if (runtime_status != RuntimeStatus::NORMAL) {
    log_count++;
    if (log_count == 100) {
      LOG_ERROR("data error");
      log_count = 0;
      ChangePlanStatus(PlanStatus::INIT);
    }

    return;
  }

  switch (status_) {
    case PlanStatus::INIT: {
    } break;
    case PlanStatus::GLOBAL_PLAN: {
      const VehiclePose cur_pose =
          ModuleManager::GetInstance()->GetRuntimeManager()->GetVehiclePose();
      path_search_ptr_->Reset();
      auto flag =
          path_search_ptr_->Search(cur_pose, target_pose_, Vec2d::Zero());
      std::cout << "flag is " << int(flag) << std::endl;
      if (flag != REACH_END) {
        ChangePlanStatus(PlanStatus::PATH_ERROR);
      } else {
        path_search_ptr_->GetPath2D(global_path_);
        ChangePlanStatus(PlanStatus::TRAJECTORY_OPT);
      }
    } break;
    case PlanStatus::TRAJECTORY_OPT:
    case PlanStatus::ROTATION_CAL:
    case PlanStatus::ROTATION:
    case PlanStatus::TRAJECTORY_TRACKING:
    case PlanStatus::FAR_FROM_PATH:
    case PlanStatus::STOP:
    case PlanStatus::REACH_SEGMENT_END:
    case PlanStatus::REACH_GOAL:
    case PlanStatus::OPT_ERROR:
    case PlanStatus::PATH_ERROR:
      /* code */
      break;

    default:
      break;
  }
}

void PlanManager::ChangePlanStatus(const PlanStatus &new_status) {
  if (status_ == new_status) {
    return;
  }

  static std::string state_str[12] = {"INIT",
                                      "GLOBAL_PLAN",
                                      "TRAJECTORY_OPT",
                                      "ROTATION_CAL",
                                      "ROTATION",
                                      "TRAJECTORY_TRACKING",
                                      "FAR_FROM_PATH",
                                      "STOP",
                                      "REACH_SEGMENT_END",
                                      "REACH_GOAL",
                                      "OPT_ERROR"
                                      "PATH_ERROR"};

  const int pre_s = int(status_);

  status_ = new_status;

  const int cur_s = int(new_status);

  LOG_INFO("plan state form {} to {}", state_str[pre_s], state_str[cur_s]);
}
}  // namespace minco_local_planner::plan_manager