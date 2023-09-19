/**
 * @Author: Xia Yunkai
 * @Date:   2023-09-05 19:00:34
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-09-05 19:16:13
 */
#include <stdint.h>

#ifndef __PLUGINS_H__
#define __PLUGINS_H__

#include <iostream>

#include "base.h"
class Dog : public Base {
 public:
  virtual void saySomething() { std::cout << "Bark" << std::endl; }
};

class Cat : public Base {
 public:
  virtual void saySomething() { std::cout << "Meow" << std::endl; }
};

class Duck : public Base {
 public:
  virtual void saySomething() { std::cout << "Quack" << std::endl; }
};

class Cow : public Base {
 public:
  virtual void saySomething() { std::cout << "Moooo" << std::endl; }
};

class Sheep : public Base {
 public:
  virtual void saySomething() { std::cout << "Baaah" << std::endl; }
};
#endif /* __PLUGIN_H__ */
