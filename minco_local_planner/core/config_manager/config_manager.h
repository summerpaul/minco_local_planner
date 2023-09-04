/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-24 15:23:28
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-09-04 17:07:24
 */
#include <stdint.h>

#ifndef __CONFIG_MANAGER_H__
#define __CONFIG_MANAGER_H__
#include "basis/base_module.h"
#include "config_data.h"
#include "utils/json_deep_loader.hpp"
namespace minco_local_planner::config_manager {

using namespace basis;
using namespace utils;
class ConfigManager : public BaseModule {
 public:
  typedef std::shared_ptr<ConfigManager> Ptr;

 public:
  ConfigManager(const std::string& path);

  virtual bool Init() override;
  virtual bool Start() override;
  virtual void Stop() override;

  bool ParseConfig();

 public:
  const LogConfig::Ptr GetLogConfig() const { return log_cfg_; }

  const RuntimeMangerConfig::Ptr GetRuntimeMangerConfig() const {
    return runtime_manager_cfg_;
  }

  const MapManagerConfig::Ptr GetMapManagerConfig() const {
    return map_manager_cfg_;
  }

  const SafetyManagerConfig::Ptr GetSafetyManagerConfig() const {
    return safety_manager_cfg_;
  }

  const PlanManagerConfig::Ptr GetPlanManagerConfig() const {
    return plan_manager_cfg_;
  }

  const AstarConfig::Ptr GetAstarConfig() const { return astar_cfg_; }

  const CarLikeKinoAstarConfig::Ptr GetCarLikeKinoAstarConfig() const {
    return car_like_kino_astar_cfg_;
  }

  const KinoAstarConfig::Ptr GetKinoAstarConfig() const {
    return kino_astar_cfg_;
  }

 private:
  bool ParseLogConfig(const Json& log_cfg_json);
  bool ParseRuntimeMangerConfig(const Json& runtime_manager_cfg_json);
  bool ParseMapManagerConfig(const Json& map_manager_cfg_json);
  bool ParseSafetyManagerConfig(const Json& safety_manager_cfg_json);
  bool ParsePlanManagerConfig(const Json& plan_manager_cfg_json);

  // // 算法相关
  bool ParseAstarConfig(const Json& astar_cfg_json);

 private:
  std::string config_file_path_;
  LogConfig::Ptr log_cfg_;
  RuntimeMangerConfig::Ptr runtime_manager_cfg_;
  MapManagerConfig::Ptr map_manager_cfg_;
  SafetyManagerConfig::Ptr safety_manager_cfg_;
  PlanManagerConfig::Ptr plan_manager_cfg_;
  AstarConfig::Ptr astar_cfg_;
  CarLikeKinoAstarConfig::Ptr car_like_kino_astar_cfg_;
  KinoAstarConfig::Ptr kino_astar_cfg_;
};
}  // namespace minco_local_planner::config_manager

#endif /* __CONFIG_MANAGER_H__ */
