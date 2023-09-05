/**
 * @Author: Xia Yunkai
 * @Date:   2023-09-05 19:00:06
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-09-05 19:00:19
 */
#include <stdint.h>

#ifndef __BASE_H__
#define __BASE_H__

class Base {
 public:
  virtual ~Base() {}
  virtual void saySomething() = 0;
};

#endif /* __BASE_H__ */
