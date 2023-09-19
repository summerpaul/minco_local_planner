/**
 * @Author: Yunkai Xia
 * @Date:   2023-09-19 10:06:56
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-09-19 10:07:36
 */
#include <stdint.h>

#ifndef __LOG_CONFIG_H__
#define __LOG_CONFIG_H__
#include <memory>
#include <string>
namespace minco_local_planner::config_manager {
    
struct LogConfig {
  typedef std::shared_ptr<LogConfig> Ptr;
  std::string log_path = "/root/log";  // 日志文件目录
  int log_type = 0;   // 日志类型，0：console_only 1:file_only 2:both
  int log_level = 0;  // 日志等级
};

}  // namespace minco_local_planner::config_manager

#endif /* __LOG_CONFIG_H__ */
