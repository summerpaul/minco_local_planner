/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-24 15:11:34
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-08-24 19:24:41
 */
#include <stdint.h>

#ifndef __CONFIG_DATA_H__
#define __CONFIG_DATA_H__
#include <string>
namespace minco_local_planner::config_manager {
  
struct LogConfig {
  std::string log_path = "/root/log";
  int log_type = 0;
  int log_level = 0;
};



}  // namespace config_manager

#endif /* __CONFIG_DATA_H__ */
