/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-24 15:45:56
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-08-29 20:11:37
 */
#include <stdint.h>

#ifndef __MODULE_MANAGER_H__
#define __MODULE_MANAGER_H__

#include "basis/logger.h"
#include "config_manager/config_manager.h"
#include "map_manager/map_manager.h"
#include "runtime_manager/runtime_manager.h"
#include "safety_manager/safety_manager.h"
#include "utils/singleton.h"
#include "utils/timer_manager.h"
#include "utils/timer_thread.h"
namespace minco_local_planner::module_manager {
using namespace basis;
using namespace config_manager;
using namespace runtime_manager;
using namespace map_manager;
using namespace safety_manager;
using namespace utils;

enum class InitStep {
  Init_ConfigManager=0,   // 初始化配置文件
  Init_Log,             // 初始化日志
  Init_RuntimeManager,  // 初始化实时数据管理
  Init_MapManager,      // 初始化地图管理
  Init_SafetyManager,   // 初始化安全管理
  Init_PlanManager,     // 初始化规划管理
  Init_Done,            // 初始化完成
  Init_Failed,          // 初始化失败

};

class ModuleManager {
 public:
  static ModuleManager *GetInstance();

  bool Init(const std::string &config_file_path);

  void Run();

 public:
  const ConfigManager::Ptr GetConfigManager() const {
    return config_manager_ptr_;
  }

  const RuntimeManager::Ptr GetRuntimeManager() const {
    return runtime_manager_ptr_;
  }

  const MapManager::Ptr GetMapManager() const { return map_manager_ptr_; }

  const SafetyManager::Ptr GetSafetyManager() const {
    return safety_manager_ptr_;
  }

 private:
  ModuleManager() = default;
  ~ModuleManager();
  ModuleManager(const ModuleManager &) = delete;
  ModuleManager &operator=(const ModuleManager &) = delete;

 private:
  void ChangeInitStep(const InitStep &next_step);

 private:
  InitStep init_step_;
  ConfigManager::Ptr config_manager_ptr_;
  RuntimeManager::Ptr runtime_manager_ptr_;
  MapManager::Ptr map_manager_ptr_;
  SafetyManager::Ptr safety_manager_ptr_;

  std::unique_ptr<TimerThread> timer_thread_;
};

}  // namespace minco_local_planner::module_manager

#endif /* __MODULE_MANAGER_H__ */
