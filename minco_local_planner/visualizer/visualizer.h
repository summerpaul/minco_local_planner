/**
 * @Author: Xia Yunkai
 * @Date:   2023-08-27 22:12:28
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-09-01 23:42:50
 */
#include <stdint.h>

#ifndef __VISUALIZER_H__
#define __VISUALIZER_H__
#include <geometry_msgs/Polygon.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
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

  void LocalGridMapVis(const Pose2d &origin, const Vec2i &dim,
                       const std::vector<int8_t> &data, const double &res,
                       const std::string &frame_id = "odom");
  void GlobalGridMapVis(const Pose2d &origin, const Vec2i &dim,
                        const std::vector<int8_t> &data, const double &res,
                        const std::string &frame_id = "odom");
  void TransformedPcdVis(const PointCloud3d &cloud,
                         const std::string frame_id = "base_link");

  void SafetyBoundingBoxesVis(const std::map<int, BoundingBox> &boxes,
                              const std::string frame_id = "base_link");
  void BezierSegmentsVis(const Path2d &bezier_segment_path,
                         const Points2d &all_control_points,
                         const std::string &frame_id = "odom");
  void GlobalPathVis(const Path2d &path, const std::string &frame_id = "odom");

 private:
  ros::NodeHandle nh_;
  ros::Publisher local_map_pub_;   // 局部栅格地图
  ros::Publisher global_map_pub_;  // 全局动态栅格地图
  ros::Publisher transformed_pcd_pub_;
  ros::Publisher safety_bounding_boxes_pub_;
  ros::Publisher bezier_segments_pub_;
  ros::Publisher global_path_pub_;
};
}  // namespace visualizer

#endif /* __VISUALIZER_H__ */
