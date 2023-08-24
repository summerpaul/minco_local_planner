/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-24 15:45:56
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-08-24 16:00:16
 */
#include <stdint.h>

#ifndef __MODULE_MANAGER_H__
#define __MODULE_MANAGER_H__

#include "basis/logger.h"
#include "config_manager/config_manager.h"
namespace minco_local_planner::module_manager {
using namespace basis;
using namespace config_manager;
enum class InitStep {
  Init_ConfigManager,   // 初始化配置文件
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

  bool Run(const std::string &config_file_path);

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
};

}  // namespace minco_local_planner::module_manager

#endif /* __MODULE_MANAGER_H__ */
