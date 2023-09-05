/**
 * @Author: Xia Yunkai
 * @Date:   2023-09-05 19:01:36
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-09-05 19:02:44
 */
#include "plugins.h"

#include <iostream>

#include "plugins.h"
#include "plugin_loader/plugin_loader.hpp"
PLUGIN_LOADER_REGISTER_CLASS(Dog, Base)
PLUGIN_LOADER_REGISTER_CLASS(Cat, Base)
PLUGIN_LOADER_REGISTER_CLASS(Duck, Base)
PLUGIN_LOADER_REGISTER_CLASS(Cow, Base)
PLUGIN_LOADER_REGISTER_CLASS(Sheep, Base)