/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-31 08:47:53
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-09-06 08:56:29
 */
#include <stdint.h>

#ifndef __PLAN_MANAGER_H__
#define __PLAN_MANAGER_H__
#include "basis/base_module.h"
#include "basis/logger.h"
#include "basis/vehicle_pose.h"
#include "config_manager/config_data.h"
#include "path_search/path_search.h"
#include "plugin_loader/plugin_loader.hpp"
namespace minco_local_planner::plan_manager {

using namespace basis;
using namespace path_search;
using namespace config_manager;

enum class PlanStatus {
  INIT = 0,         // 初始化状态
  GLOBAL_PLAN = 1,  // 优化失败后使用混合A星规划（先不适用）
  TRAJECTORY_OPT = 2,       // 轨迹优化
  ROTATION_CAL = 3,         // 设置旋转参数
  ROTATION = 4,             // 轨迹跟踪前的自旋
  TRAJECTORY_TRACKING = 5,  // 轨迹跟踪
  FAR_FROM_PATH = 6,        // 车辆远离轨迹
  STOP = 7,                 // 停止
  REACH_SEGMENT_END = 8,    // 到达段的终点
  REACH_GOAL = 9,           // 达到目标点
  OPT_ERROR = 10,           // 轨迹优化错误
  PATH_ERROR = 11           // 初始轨迹错误，在bezier段为空时出现

};

class PlanManager : public BaseModule {
 public:
  typedef std::shared_ptr<PlanManager> Ptr;
  PlanManager();
  ~PlanManager();

  virtual bool Init() override;
  virtual bool Start() override;
  //   用于取消导航，清空数据
  virtual void Stop() override;
  // 用于全局规划
  void SetTargetPose(const VehiclePose &pose);

  const Path2d &GetGlobalPath() const { return global_path_; }

 private:
  void ReplanFSMTimer();

  void ChangePlanStatus(const PlanStatus &new_status);
  // 用于清空规划过程中的数据
  void Clear();

 private:
  PlanStatus status_;
  VehiclePose target_pose_;
  PlanManagerConfig::Ptr cfg_;
  Path2d global_path_;
  std::unique_ptr<plugin_loader::PluginLoader> path_search_plugin_loader_;
  PathSearch::Ptr path_search_ptr_;  // 全局路径规划器
};
}  // namespace minco_local_planner::plan_manager

#endif /* __PLAN_MANAGER_H__ */
