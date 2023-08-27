/**
 * @Author: Xia Yunkai
 * @Date:   2023-08-24 21:22:24
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-08-27 23:09:15
 */
#include "map_manager.h"

#include <iostream>

#include "utils/singleton.h"
#include "utils/timer_manager.h"

namespace minco_local_planner::map_manager {

MapManager::MapManager() : BaseModule("MapManager") {}
MapManager::~MapManager() {}

bool MapManager::Init() { return true; }
bool MapManager::Start() {
  Singleton<TimerManager>()->Schedule(
      100, std::bind(&MapManager::LaserScanTransformTimer, this));

  Singleton<TimerManager>()->Schedule(
      100, std::bind(&MapManager::GenerateGridMapTimer, this));
  return true;
}
void MapManager::Stop() {}

void MapManager::LaserScanTransformTimer() {
  LOG_INFO("in LaserScanTransformTimer");
}

void MapManager::GenerateGridMapTimer() { LOG_INFO("in GenerateGridMapTimer"); }

}  // namespace minco_local_planner::map_manager