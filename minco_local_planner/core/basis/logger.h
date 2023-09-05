/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-24 13:12:53
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-09-05 22:11:23
 */
#include <stdint.h>

#ifndef __LOGGER_H__
#define __LOGGER_H__
#include <spdlog/async.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <time.h>

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
namespace minco_local_planner::basis {
static inline int NowDateToInt() {
  time_t now;
  time(&now);

  // choose thread save version in each platform
  tm p;
#ifdef _WIN32
  localtime_s(&p, &now);
#else
  localtime_r(&now, &p);
#endif  // _WIN32
  int now_date = (1900 + p.tm_year) * 10000 + (p.tm_mon + 1) * 100 + p.tm_mday;
  return now_date;
}

static inline int NowTimeToInt() {
  time_t now;
  time(&now);
  // choose thread save version in each platform
  tm p;
#ifdef _WIN32
  localtime_s(&p, &now);
#else
  localtime_r(&now, &p);
#endif  // _WIN32

  int now_int = p.tm_hour * 10000 + p.tm_min * 100 + p.tm_sec;
  return now_int;
}

enum class LoggerType { CONSOLE_ONLY = 0, FILE_ONLY = 1, SPLITTER = 2 };

class Logger {
 public:
  static Logger *GetInstance() {
    static Logger logger;
    return &logger;
  }

  std::shared_ptr<spdlog::logger> GetLogger() { return logger_ptr_; }

  bool GetInitFlag() { return b_init_; }

  bool Init(const std::string &log_dir, const std::string &logger_name_prefix,
            const int &log_level = 0, const int &log_type = 0) {
    if (b_init_) {
      return true;
    }

    const auto spd_log_level = spdlog::level::level_enum(log_level);
    const std::string pattern = "%Y-%m-%d %H:%M:%S.%e [%l] [%s %#] %v";

    try {
      int date = NowDateToInt();
      int time = NowTimeToInt();
      const std::string logger_name = logger_name_prefix +
                                      std::to_string(date) + "_" +
                                      std::to_string(time);
      const std::string log_path = log_dir + "/" + logger_name + ".log";
      CreateLogger(LoggerType(log_type), logger_name, log_path);
      logger_ptr_->set_level(spd_log_level);
      logger_ptr_->flush_on(spd_log_level);
      logger_ptr_->set_pattern(pattern);
      spdlog::register_logger(logger_ptr_);
      spdlog::set_default_logger(spdlog::get(logger_name));
      b_init_ = true;
    } catch (const spdlog::spdlog_ex &ex) {
      b_init_ = false;
      std::cout << "Log initialization failed: " << ex.what() << std::endl;
    }

    return b_init_;
  }

 private:
  void CreateLogger(const LoggerType &log_type, const std::string &name,
                    const std::string &path) {
    std::vector<spdlog::sink_ptr> sinks;

    if (log_type == LoggerType::CONSOLE_ONLY) {
      auto console_sink =
          std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
      sinks.push_back(console_sink);

      std::cout << "log use console only " << std::endl;
    } else if (log_type == LoggerType::FILE_ONLY) {
      auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
          path.c_str(), true);
      std::cout << "log use file only " << std::endl;
      sinks.push_back(file_sink);
    } else {
      auto console_sink =
          std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
      auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
          path.c_str(), true);
      sinks.push_back(console_sink);
      sinks.push_back(file_sink);

      std::cout << "log use  console and file " << std::endl;
    }
    logger_ptr_ =
        std::make_shared<spdlog::logger>(name, begin(sinks), end(sinks));
  }

 private:
  Logger() = default;
  ~Logger() { spdlog::drop_all(); }
  Logger(const Logger &) = delete;
  Logger &operator=(const Logger &) = delete;

 private:
  std::shared_ptr<spdlog::logger> logger_ptr_;
  bool b_init_ = false;
};

#define LOG_TRACE(...)                                         \
  SPDLOG_LOGGER_CALL(Logger::GetInstance()->GetLogger().get(), \
                     spdlog::level::trace, __VA_ARGS__)

#define LOG_DEBUG(...)                                         \
  SPDLOG_LOGGER_CALL(Logger::GetInstance()->GetLogger().get(), \
                     spdlog::level::debug, __VA_ARGS__)

#define LOG_INFO(...)                                          \
  SPDLOG_LOGGER_CALL(Logger::GetInstance()->GetLogger().get(), \
                     spdlog::level::info, __VA_ARGS__)

#define LOG_WARN(...)                                          \
  SPDLOG_LOGGER_CALL(Logger::GetInstance()->GetLogger().get(), \
                     spdlog::level::warn, __VA_ARGS__)

#define LOG_ERROR(...)                                         \
  SPDLOG_LOGGER_CALL(Logger::GetInstance()->GetLogger().get(), \
                     spdlog::level::err, __VA_ARGS__)
}  // namespace minco_local_planner::basis

#endif /* __LOGGER_H__ */
