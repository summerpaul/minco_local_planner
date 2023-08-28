/**
 * @Author: Xia Yunkai
 * @Date:   2023-08-27 22:12:32
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-08-28 10:37:32
 */
#include "visualizer.h"

#include <iostream>
using namespace std;

namespace visualizer {

Visualizer::Visualizer() {
  grid_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("grid_map", 1);
  transformed_pcd_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("transformed_pcd", 1);
}

void Visualizer::GridMapVis(const Pose2d &origin, const Vec2i &dim,
                            const std::vector<int8_t> &data, const double &res,
                            const std::string &frame_id) {
  nav_msgs::OccupancyGrid grid_map_ros;
  grid_map_ros.header.frame_id = frame_id;
  geometry_msgs::Pose pose;
  pose.position.x = origin.x();
  pose.position.y = origin.y();
  pose.orientation.w = 1;
  grid_map_ros.info.map_load_time = ros::Time::now();
  grid_map_ros.header.stamp = ros::Time::now();
  grid_map_ros.data = data;
  grid_map_ros.info.resolution = res;
  grid_map_ros.info.width = dim.x();
  grid_map_ros.info.height = dim.y();
  grid_map_ros.info.origin = pose;

  grid_map_pub_.publish(grid_map_ros);
}

void Visualizer::TransformedPcdVis(const PointCloud3d &cloud,
                                   const std::string frame_id) {
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.frame_id = frame_id;

  transformed_pcd_pub_.publish(cloud_msg);
}
}  // namespace visualizer