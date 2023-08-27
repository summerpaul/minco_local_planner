/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-24 15:11:34
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-08-25 11:02:04
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

struct RuntimeMangerConfig {
  // 检查数据异常的频率
  double check_sleep_time = 0.1;
  //   数据异常的时间间隔
  double message_wait_time = 0.5;
};

}  // namespace minco_local_planner::config_manager

#endif /* __CONFIG_DATA_H__ */
