/**
 * @Author: Yunkai Xia
 * @Date:   2023-09-19 10:08:03
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-09-19 10:08:25
 */
#include <stdint.h>

#ifndef __RUNTIME_MANAGER_CONFIG_H__
#define __RUNTIME_MANAGER_CONFIG_H__

#include <memory>
#include <string>
namespace minco_local_planner::config_manager {

struct RuntimeMangerConfig {
  typedef std::shared_ptr<RuntimeMangerConfig> Ptr;
  double check_sleep_time = 0.1;   // 检查数据异常的频率
  double message_wait_time = 0.5;  //   数据异常的时间间隔
};
}  // namespace minco_local_planner::config_manager

#endif /* __RUNTIME_MANAGER_CONFIG_H__ */
