/**
 * @Author: Xia Yunkai
 * @Date:   2023-08-27 22:12:28
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-08-30 10:59:10
 */
#include <stdint.h>

#ifndef __VISUALIZER_H__
#define __VISUALIZER_H__
#include <geometry_msgs/Polygon.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include "basis/data_type.h"
#include "basis/pcl_types.h"
#include "safety_manager/bounding_box.h"
#include "vis_tools.h"
namespace visualizer {

using namespace minco_local_planner::basis;
using namespace minco_local_planner::safety_manager;
class Visualizer {
 public:
  Visualizer();

  void GridMapVis(const Pose2d &origin, const Vec2i &dim,
                  const std::vector<int8_t> &data, const double &res,
                  const std::string &frame_id = "odom");
  void TransformedPcdVis(const PointCloud3d &cloud,
                         const std::string frame_id = "base_link");

  void SafetyBoundingBoxesVis(const std::map<int, BoundingBox> &boxes,
                              const std::string frame_id = "base_link");
  void BezierSegmentsVis(const Path2d &bezier_segment_path,
                         const Points2d &all_control_points,
                         const std::string &frame_id = "odom");

 private:
  ros::NodeHandle nh_;
  ros::Publisher grid_map_pub_;
  ros::Publisher transformed_pcd_pub_;
  ros::Publisher safety_bounding_boxes_pub_;
  ros::Publisher bezier_segments_pub_;
};
}  // namespace visualizer

#endif /* __VISUALIZER_H__ */
