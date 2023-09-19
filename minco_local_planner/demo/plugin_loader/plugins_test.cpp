/**
 * @Author: Xia Yunkai
 * @Date:   2023-09-05 19:03:49
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-09-05 19:17:12
 */
#include "base.h"
#include "plugin_loader/multi_library_plugin_loader.hpp"
#include "plugin_loader/plugin_loader.hpp"

// Run all the tests that were declared with TEST()
int main() {
  // console_bridge::setLogLevel(console_bridge::CONSOLE_BRIDGE_LOG_DEBUG);

  plugin_loader::PluginLoader loader1("../libplugins.so", false);

  auto names = loader1.getAvailableClasses<Base>();

  for (const auto& name : names) {
    loader1.createInstance<Base>(name)->saySomething();
  }

  return 0;
}