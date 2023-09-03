/**
 * @Author: Xia Yunkai
 * @Date:   2023-09-02 16:54:11
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-09-02 22:36:16
 */
#include <stdint.h>

#ifndef __CAR_LIKE_KINO_ASTAR_H__
#define __CAR_LIKE_KINO_ASTAR_H__

#include <map>

#include "path_search.h"
namespace minco_local_planner::path_search {
class CarLikeKinoAstar : public PathSearch {
 public:
  CarLikeKinoAstar();
  ~CarLikeKinoAstar();
  virtual int Search(const VehiclePose& start_pos, const VehiclePose& end_pos,
                     const Vec2d& init_ctrl) override;

  virtual void Reset() override;
  virtual bool Init() override;
  virtual bool Start() override;
  virtual void Stop() override;
  virtual void GetPath2D(Path2d& path) override;

 private:
  CarLikeKinoAstarConfig::Ptr cfg_;
  std::vector<MiddleNodePtr> middle_node_pool_;
  std::vector<MiddleNodePtr> middle_nodes_;
  std::multimap<double, MiddleNodePtr> openSet_;
  Path2d last_path_pos_, last_path_pos_temp_;
  int use_time_node_num_;

  MiddleNodeHashTable<MiddleNodePtr> expanded_middle_nodes_;
  double yaw_origin_ = -M_PI;
};
}  // namespace minco_local_planner::path_search
#endif /* __CAR_LIKE_KINO_ASTAR_H__ */
