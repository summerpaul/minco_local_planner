/**
 * @Author: YunKai Xia
 * @Date:   2023-08-26 14:59:01
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-08-27 21:10:23
 */
#include <stdint.h>

#ifndef __TIMER_THREAD_H__
#define __TIMER_THREAD_H__
// 单线程形式的计时器
#include <atomic>
#include <functional>
#include <thread>

namespace minco_local_planner::utils {
class TimerThread {
 public:
  TimerThread() : active_(false), period_(0), repeat_(-1) {}
  TimerThread(int repeat) : active_(false), period_(0), repeat_(repeat) {}
  ~TimerThread() { Stop(); }

  template <typename F, typename... Args>
  void RegisterCallback(int milliseconds, F&& f, Args&&... args) {
    period_ = milliseconds;

    callback_ = std::bind(std::forward<F>(f), std::forward<Args>(args)...);
  }

  void Start() {
    if (active_.load()) {
      // 已经激活，不需要重新设置
      return;
    }

    active_.store(true);

    thread_ = std::thread([&]() {
      if (repeat_ < 0) {
        while (active_.load()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(period_));
          if (!active_.load()) return;

          callback_();
        }
      } else {
        while (repeat_ > 0) {
          if (!active_.load()) return;

          std::this_thread::sleep_for(std::chrono::milliseconds(period_));
          if (!active_.load()) return;
          callback_();
          repeat_--;
        }
      }
    });
  }

  void Stop() {
    active_.store(false);
    if (thread_.joinable()) {
      thread_.join();
    }
  }

 private:
  std::thread thread_;
  std::atomic<bool> active_;
  std::function<void()> callback_;  // 定时器的回调函数
  int period_;                      // 定时器的时间间隔
  int repeat_;                      // 触发的次数 -1表示永久执行
};
}  // namespace minco_local_planner::utils
#endif /* __TIMER_THREAD_H__ */
