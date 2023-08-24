/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-24 17:23:47
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-08-24 17:43:15
 */
#include <stdint.h>

#ifndef __RUNTIME_MANAGER_H__
#define __RUNTIME_MANAGER_H__

#include "basis/base_module.h"
#include "basis/data_type.h"
#include "basis/logger.h"

namespace minco_local_planner::runtime_manager {

using namespace basis;
class RunTimeManager : public BaseModule {
 public:
  typedef std::shared_ptr<RunTimeManager> Ptr;
  RunTimeManager();
  ~RunTimeManager();

  virtual bool Init() override;
  virtual bool Start() override;
  virtual void Stop() override;

 private:
};
}  // namespace minco_local_planner::runtime_manager

#endif /* __RUNTIME_MANAGER_H__ */
