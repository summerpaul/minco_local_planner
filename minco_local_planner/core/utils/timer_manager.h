/**
 * @Author: YunKai Xia
 * @Date:   2023-08-26 15:44:08
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-08-27 21:16:14
 */
#include <stdint.h>

#ifndef __TIMER_MANAGER_H__
#define __TIMER_MANAGER_H__

#include <chrono>
#include <functional>
#include <map>

namespace minco_local_planner::utils {

class Timer {
  friend class TimerManager;

 public:
  Timer() : period_(0), repeat_(-1) { time_ = Now(); }
  Timer(int repeat) : period_(0), repeat_(repeat) { time_ = Now(); }
  ~Timer() {}

  // 注册函数
  template <typename F, typename... Args>
  void RegisterCallback(int milliseconds, F&& f, Args&&... args) {
    period_ = milliseconds;

    callback_ = std::bind(std::forward<F>(f), std::forward<Args>(args)...);
  }

  static int64_t Now() {
    // 获取当前时间戳
    auto now = std::chrono::system_clock::now();
    // 将时间戳转换成毫秒数
    auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);

    return now_ms.time_since_epoch().count();
  }

  void OnTimer() {
    if (!callback_ || repeat_ == 0) return;

    callback_();
    time_ += period_;
    if (repeat_ > 0) repeat_--;
  }

 private:
  int64_t time_;  // 定时器触发的时间点，单位：毫秒

  std::function<void()> callback_;
  int period_;  // 定时器触发的周期，单位：毫秒
  int repeat_;  // 定时器触发的次数，-1表示无限触发
};

class TimerManager {
 public:
  TimerManager() = default;

  ~TimerManager() = default;

 public:
  template <typename F, typename... Args>
  void Schedule(int milliseconds, F&& f, Args&&... args) {
    Schedule(milliseconds, -1, std::forward<F>(f), std::forward<Args>(args)...);
  }

  template <typename F, typename... Args>
  void Schedule(int milliseconds, int repeat, F&& f, Args&&... args) {
    Timer t(repeat);
    t.RegisterCallback(milliseconds, std::forward<F>(f),
                       std::forward<Args>(args)...);

    timers_.insert(std::make_pair(t.time_, t));
  }

  void Update() {
    if (timers_.empty()) {
      return;
    }

    int64_t now = Timer::Now();

    for (auto it = timers_.begin(); it != timers_.end();) {
      // 后面的定时器更晚，直接退出
      if (it->first > now) {
        return;
      }
      // 启动定时器中的回调函数
      it->second.OnTimer();

      //   缓存当前定时器
      Timer t = it->second;

      //   删除之前的定时器,返回下一个定时器
      it = timers_.erase(it);

      //   判断缓存的定时器还需触发的次数.
      if (t.repeat_ != 0) {
        auto new_it = timers_.insert(std::make_pair(t.time_, t));
        // 判断当前定时器队列是否为空||缓存的定时器时间早于当前定时器

        if (it == timers_.end() || new_it->first < it->first) {
          it = new_it;
        }
      }
    }
  }

 private:
  // 有序排序
  std::multimap<int64_t, Timer> timers_;
};

}  // namespace minco_local_planner::utils
#endif /* __TIMER_MANAGER_H__ */
