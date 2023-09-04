/**
 * @Author: Yunkai Xia
 * @Date:   2023-09-04 15:31:00
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-09-04 15:32:42
 */

#ifndef __SCOPE_EXIT_HPP__
#define __SCOPE_EXIT_HPP__
#include <functional>

#include "defines.h"
namespace minco_local_planner::basis {

//! 退出区域的时候执行的动作
class ScopeExitActionGuard {
  using ScopeExitFunc = std::function<void()>;
  ScopeExitFunc func_;

 public:
  ScopeExitActionGuard(const ScopeExitFunc &func) : func_(func) {}

  NONCOPYABLE(ScopeExitActionGuard);
  IMMOVABLE(ScopeExitActionGuard);

  ~ScopeExitActionGuard() {
    if (func_) func_();
  }

  void cancel() { func_ = nullptr; }
};

}  // namespace minco_local_planner::basis

#define _ScopeExitActionName_1(line)               \
  minco_local_planner::basis::ScopeExitActionGuard \
      _scope_exit_action_guard_##line
#define _ScopeExitActionName_0(line) _ScopeExitActionName_1(line)

#define SetScopeExitAction(...) _ScopeExitActionName_0(__LINE__)(__VA_ARGS__)

#endif /* __SCOPE_EXIT_HPP__ */
