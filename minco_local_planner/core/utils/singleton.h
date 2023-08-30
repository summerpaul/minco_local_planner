/**
 * @Author: YunKai Xia
 * @Date:   2023-08-26 16:31:03
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-08-27 21:08:15
 */
#include <stdint.h>

#ifndef __SINGLETON_H__
#define __SINGLETON_H__
namespace minco_local_planner::utils {
template <class T>
T* Singleton() {
  static T instance;
  return &instance;
}
}  // namespace minco_local_planner::utils

#endif /* __SINGLETON_H__ */
