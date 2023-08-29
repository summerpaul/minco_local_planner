/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-24 15:23:28
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-08-29 18:49:40
 */
#include <stdint.h>

#ifndef __CONFIG_MANAGER_H__
#define __CONFIG_MANAGER_H__
#include "basis/base_module.h"
#include "config_data.h"

#if defined(__GNUC__) && (defined(__x86_64__) || defined(__i386__))
#include <jsoncpp/json/json.h>
#elif defined(__GNUC__) && defined(__ARM_NEON)
#include <json/json.h>
#endif

namespace minco_local_planner::config_manager {

using namespace basis;

class ConfigManager : public BaseModule {
 public:
  typedef std::shared_ptr<ConfigManager> Ptr;

 public:
  ConfigManager(const std::string& path);

  virtual bool Init() override;
  virtual bool Start() override;
  virtual void Stop() override;

 public:
  const LogConfig& GetLogConfig() const { return log_cfg_; }
  const RuntimeMangerConfig& GetRuntimeMangerConfig() const {
    return runtime_manager_cfg_;
  }

  const MapManagerConfig& GetMapManagerConfig() const {
    return map_manager_cfg_;
  }
  const SafetyManagerConfig& GetSafetyManagerConfig() const {
    return safety_manager_cfg_;
  }

 private:
  bool ParseLogConfig(const Json::Value& log_cfg_json);
  bool ParseRuntimeMangerConfig(const Json::Value& runtime_manager_cfg_json);

 private:
  std::string config_file_path_;
  LogConfig log_cfg_;
  RuntimeMangerConfig runtime_manager_cfg_;
  MapManagerConfig map_manager_cfg_;
  SafetyManagerConfig safety_manager_cfg_;
};
}  // namespace minco_local_planner::config_manager

#endif /* __CONFIG_MANAGER_H__ */
