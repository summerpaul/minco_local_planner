/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-24 15:23:28
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-08-25 13:34:56
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
    return runtime_manager_cfg;
  }

 private:
  bool ParseLogConfig(const Json::Value& log_cfg_json);
  bool ParseRuntimeMangerConfig(const Json::Value& runtime_manager_cfg_json);

 private:
  std::string config_file_path_;
  LogConfig log_cfg_;
  RuntimeMangerConfig runtime_manager_cfg;
};
}  // namespace minco_local_planner::config_manager

#endif /* __CONFIG_MANAGER_H__ */
