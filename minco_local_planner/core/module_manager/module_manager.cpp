/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-24 15:46:03
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-08-31 09:50:14
 */
#include "module_manager.h"

#include <iostream>

namespace minco_local_planner::module_manager {

ModuleManager::~ModuleManager() {}

ModuleManager* ModuleManager::GetInstance() {
  static ModuleManager module_manager;
  return &module_manager;
}

bool ModuleManager::Init(const std::string& config_file_path) {
  config_manager_ptr_.reset(new ConfigManager(config_file_path));
  bool init_flag{false};

  init_step_ = InitStep::Init_ConfigManager;
  while (init_step_ != InitStep::Init_Done &&
         init_step_ != InitStep::Init_Failed) {
    switch (init_step_) {
      // step1 初始化文件
      case InitStep::Init_ConfigManager: {
        init_flag = config_manager_ptr_->Run();
        init_flag ? ChangeInitStep(InitStep::Init_Log)
                  : ChangeInitStep(InitStep::Init_Failed);
      } break;
      // step2 初始化日志
      case InitStep::Init_Log: {
        const LogConfig log_cfg = config_manager_ptr_->GetLogConfig();
        if (!Logger::GetInstance()->GetInitFlag()) {
          Logger::GetInstance()->Init(log_cfg.log_path, "sunny_local_planner",
                                      log_cfg.log_level, log_cfg.log_type);

          LOG_INFO("START_LOCAL_PLANNER_LOG---------------");
        }
        ChangeInitStep(InitStep::Init_RuntimeManager);
      } break;
      case InitStep::Init_RuntimeManager: {
        runtime_manager_ptr_.reset(new RuntimeManager);

        init_flag = runtime_manager_ptr_->Run();
        init_flag ? ChangeInitStep(InitStep::Init_MapManager)
                  : ChangeInitStep(InitStep::Init_Failed);
      } break;

      case InitStep::Init_MapManager: {
        map_manager_ptr_.reset(new MapManager);
        init_flag = map_manager_ptr_->Run();
        init_flag ? ChangeInitStep(InitStep::Init_SafetyManager)
                  : ChangeInitStep(InitStep::Init_Failed);
      } break;
      case InitStep::Init_SafetyManager: {
        safety_manager_ptr_.reset(new SafetyManager);
        init_flag = safety_manager_ptr_->Run();
        init_flag ? ChangeInitStep(InitStep::Init_PlanManager)
                  : ChangeInitStep(InitStep::Init_Failed);
      } break;
      case InitStep::Init_PlanManager: {
        plan_manager_ptr_.reset(new PlanManager);
        init_flag = plan_manager_ptr_->Run();
        init_flag ? ChangeInitStep(InitStep::Init_Done)
                  : ChangeInitStep(InitStep::Init_Failed);
      }
      case InitStep::Init_Done: {
        LOG_INFO("Init_Done");
        break;
      }
      case InitStep::Init_Failed: {
        LOG_ERROR("Init_Failed");
        break;
      }
    }
  }
  return init_flag;
}

void ModuleManager::Run() {
  timer_thread_.reset(new TimerThread);

  timer_thread_->RegisterCallback(
      10, [&]() { Singleton<TimerManager>()->Update(); });
  timer_thread_->Start();
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