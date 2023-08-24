/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-24 13:08:35
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-08-24 16:44:07
 */
#include <stdint.h>

#ifndef __BASE_MODULE_H__
#define __BASE_MODULE_H__
#include <iostream>
#include <memory>
#include <string>
namespace minco_local_planner::basis {

class BaseModule {
 public:
  BaseModule(const std::string& module_name = "") : module_name_(module_name) {}
  virtual ~BaseModule() {}

  const std::string& Name() const { return module_name_; }

  virtual bool Init() = 0;
  virtual bool Start() = 0;
  virtual void Stop() = 0;

  bool Run() {
    if (!Init()) {
      std::cout << module_name_ << " failed to init" << std::endl;
      return false;
    }
    std::cout << module_name_ << " success to init" << std::endl;
    if (!Start()) {
      std::cout << module_name_ << " failed to start" << std::endl;
      return false;
    }
    return true;
  }

 protected:
  std::string module_name_;
};

}  // namespace minco_local_planner::basis

#endif /* __BASE_MODULE_H__ */
