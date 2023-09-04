/**
 * @Author: Xia Yunkai
 * @Date:   2023-09-04 22:34:20
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-09-04 22:35:36
 */
#include <stdint.h>

#ifndef __TYPES_H__
#define __TYPES_H__

#include <unordered_map>

#include "basis/data_type.h"

namespace minco_local_planner::segments {

using namespace basis;

struct LaneNode {
  int id;
  Pose2d pose;
};
struct Operate {
  int node;     // 目标点
  int op_code;  // 操作码  0 默认  无定义    1 顶升   2下降
};

struct LaneInfo {
  int avoid_level;  // 避障等级
  int motion_type;  // 前进1 倒退-1
  int heading;      // 全向机器人
  double speed;     // 速度
};

struct LaneShape {
  Vec2d p1;
  Vec2d p2;
  double length;
  double width;  // 2023 6月12 增加局部路径规划相关的路宽
};

struct Lane {
  int id;
  LaneNode start;
  LaneNode end;
  LaneShape shape;
  LaneInfo info;
};

typedef std::vector<Lane> Lanes;
typedef std::unordered_map<int, LaneNode> MapNodes;
typedef std::unordered_map<int, Lane> MapLanes;
}  // namespace minco_local_planner::segments

#endif /* __TYPES_H__ */
