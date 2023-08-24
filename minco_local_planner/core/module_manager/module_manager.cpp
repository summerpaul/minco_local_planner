/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-24 15:46:03
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-08-24 16:29:40
 */
#include "module_manager.h"

#include <iostream>

namespace minco_local_planner::module_manager {

ModuleManager::~ModuleManager() {}

ModuleManager* ModuleManager::GetInstance() {
  static ModuleManager module_manager;
  return &module_manager;
}

bool ModuleManager::Run(const std::string& config_file_path) {
  config_manager_ptr_ = std::make_shared<ConfigManager>(config_file_path);
  bool init_flag{false};
  init_step_ = InitStep::Init_ConfigManager;
  while (init_step_ != InitStep::Init_Done &&
         init_step_ != InitStep::Init_Failed) {
    switch (init_step_) {
      case InitStep::Init_ConfigManager: {
        init_flag = config_manager_ptr_->Run();
        init_flag ? ChangeInitStep(InitStep::Init_Log)
                  : ChangeInitStep(InitStep::Init_Failed);
      } break;
      case InitStep::Init_Log: {
        const LogConfig log_cfg = config_manager_ptr_->GetLogConfig();
        if (!Logger::GetInstance()->GetInitFlag()) {
          Logger::GetInstance()->Init(log_cfg.log_path, "sunny_local_planner",
                                      log_cfg.log_level, log_cfg.log_type);

          LOG_INFO("START_LOCAL_PLANNER_LOG---------------");
        }
        ChangeInitStep(InitStep::Init_Done);
      } break;
    }
  }
  return init_flag;
}

void ModuleManager::ChangeInitStep(const InitStep& next_step) {
  static std::string state_str[8] = {"Init_ConfigManager",  "Init_Log",
                                     "Init_RuntimeManager", "Init_MapManager",
                                     "Init_SafetyManager",  "Init_PlanManager",
                                     "Init_Done",           "Init_Failed"};
  int pre_s = int(init_step_);
  init_step_ = next_step;
  if (Logger::GetInstance()->GetInitFlag()) {
    LOG_INFO("init_step_ form {} to {}", state_str[pre_s],
             state_str[int(next_step)]);
  } else {
    std::cout << "init_step_ form " << state_str[pre_s] << " to "
              << state_str[int(next_step)] << std::endl;
  }
}

}  // namespace minco_local_planner::module_manager