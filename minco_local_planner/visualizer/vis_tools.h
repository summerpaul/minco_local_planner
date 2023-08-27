/**
 * @Author: Yunkai Xia
 * @Date:   2023-01-29 15:56:38
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-08-27 22:13:25
 */
#include <stdint.h>

#ifndef __VIS_TOOLS_H__
#define __VIS_TOOLS_H__
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Core>

#include "color.h"
using std_msgs::ColorRGBA;
using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;
namespace visualizer {

inline geometry_msgs::Pose defaultPose() {
  geometry_msgs::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
  return pose;
}

inline geometry_msgs::Pose pose2d(const double &x, const double &y,
                                  const double &theta) {
  double yaw{theta}, pitch{0}, roll{0};
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = 0.0;
  pose.orientation.x = cy * cp * sr - sy * sp * cr;
  pose.orientation.y = sy * cp * sr + cy * sp * cr;
  pose.orientation.z = sy * cp * cr - cy * sp * sr;
  pose.orientation.w = cy * cp * cr + sy * sp * sr;
  return pose;
}
inline Marker newMaker(const geometry_msgs::Vector3 &scale,
                       const geometry_msgs::Pose &pose, const std::string &ns,
                       const int32_t &id, const ColorRGBA &color,
                       const std::string &frame_id, const int32_t &type) {
  Marker marker;
  // Set marker frame and timestamp.
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.
  marker.ns = ns;
  marker.id = id;

  // Set the marker type.
  marker.type = type;

  // Set the marker action.
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.
  marker.pose = pose;

  // Set the scale and color of the marker.
  marker.scale = scale;
  marker.color = color;
  return marker;
}

inline Marker newLineStrip(const double &scale, const std::string &ns,
                           const int32_t &id, const ColorRGBA &color,
                           const std::string &frame_id) {
  geometry_msgs::Vector3 vec_scale;
  vec_scale.x = scale;
  vec_scale.y = 1.0;
  vec_scale.z = 1.0;
  return newMaker(vec_scale, defaultPose(), ns, id, color, frame_id,
                  visualization_msgs::Marker::LINE_STRIP);
}

inline Marker newArrow(const geometry_msgs::Vector3 &scale,
                       const geometry_msgs::Pose &pose, const std::string &ns,
                       const int32_t &id, const ColorRGBA &color,
                       const std::string &frame_id) {
  return newMaker(scale, pose, ns, id, color, frame_id,
                  visualization_msgs::Marker::ARROW);
}

inline Marker newCube(const geometry_msgs::Vector3 &scale,
                      const geometry_msgs::Pose &pose, const std::string &ns,
                      const int32_t &id, const ColorRGBA &color,
                      const std::string &frame_id) {
  return newMaker(scale, pose, ns, id, color, frame_id,
                  visualization_msgs::Marker::CUBE);
}

inline Marker newSphereList(const double &scale, const std::string &ns,
                            const int32_t &id, const ColorRGBA &color,
                            const std::string &frame_id) {
  geometry_msgs::Vector3 vec_scale;
  vec_scale.x = scale;
  vec_scale.y = scale;
  vec_scale.z = scale;
  return newMaker(vec_scale, defaultPose(), ns, id, color, frame_id,
                  visualization_msgs::Marker::SPHERE_LIST);
}

inline Marker newText(const double &scale, const geometry_msgs::Pose &pose,
                      const std::string &ns, const int32_t &id,
                      const ColorRGBA &color, const std::string &frame_id) {
  geometry_msgs::Vector3 vec_scale;
  vec_scale.x = 1.0;
  vec_scale.y = 1.0;
  vec_scale.z = scale;
  return newMaker(vec_scale, pose, ns, id, color, frame_id,
                  visualization_msgs::Marker::TEXT_VIEW_FACING);
}

inline Marker newCircle(const double &radius, const double &center_x,
                        const double &center_y, const std::string &ns,
                        const int32_t &id, const ColorRGBA &color,
                        const std::string &frame_id) {
  geometry_msgs::Vector3 vec_scale;
  vec_scale.x = radius * 2;
  vec_scale.y = radius * 2;
  vec_scale.z = 0.1;
  geometry_msgs::Pose pose = defaultPose();
  pose.position.x = center_x;
  pose.position.y = center_y;
  return newMaker(vec_scale, pose, ns, id, color, frame_id,
                  visualization_msgs::Marker::CYLINDER);
}

inline Marker newRectangle(const double &length, const double &width,
                           const double &scale, const geometry_msgs::Pose &pose,
                           const std::string &ns, const int32_t &id,
                           const ColorRGBA &color,
                           const std::string &frame_id) {
  Marker rectangle = newLineStrip(scale, ns, id, color, frame_id);

  const double x = pose.position.x;
  const double y = pose.position.y;
  const double yaw = 2.0 * atan2(pose.orientation.z, pose.orientation.w);
  // 切向向量
  const double k = tan(yaw);
  Eigen::Vector2d t_vec(1, k);
  t_vec.normalize();
  Eigen::Vector2d n_vec(1, -1 / k);
  n_vec.normalize();
  // p3      p2
  //    pos
  // p0      p1
  Eigen::Vector2d pos(x, y);
  std::vector<Eigen::Vector2d> pts;
  Eigen::Vector2d p0 = pos + 0.5 * width * n_vec - 0.5 * length * t_vec;
  Eigen::Vector2d p1 = pos - 0.5 * width * n_vec - 0.5 * length * t_vec;
  Eigen::Vector2d p2 = p1 + length * t_vec;
  Eigen::Vector2d p3 = p0 + length * t_vec;

  pts.emplace_back(p0);
  pts.emplace_back(p1);
  pts.emplace_back(p2);
  pts.emplace_back(p3);
  pts.emplace_back(p0);
  geometry_msgs::Point point;
  for (auto pt : pts) {
    point.x = pt.x();
    point.y = pt.y();
    point.z = 0;
    rectangle.points.emplace_back(point);
  }

  return rectangle;
}

}  // namespace visualizer

#endif /* __VIS_TOOLS_H__ */
