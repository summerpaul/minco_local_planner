/**
 * @Author: Xia Yunkai
 * @Date:   2023-08-27 22:12:28
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-08-28 10:29:59
 */
#include <stdint.h>

#ifndef __VISUALIZER_H__
#define __VISUALIZER_H__
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "basis/data_type.h"
#include "basis/pcl_types.h"
#include "vis_tools.h"
namespace visualizer {

using namespace minco_local_planner::basis;
class Visualizer {
 public:
  Visualizer();

  void GridMapVis(const Pose2d &origin, const Vec2i &dim,
                  const std::vector<int8_t> &data, const double &res,
                  const std::string &frame_id = "base_link");
  void TransformedPcdVis(const PointCloud3d &cloud,
                         const std::string frame_id = "base_link");

 private:
  ros::NodeHandle nh_;
  ros::Publisher grid_map_pub_;
  ros::Publisher transformed_pcd_pub_;
};
}  // namespace visualizer

#endif /* __VISUALIZER_H__ */
