/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-24 14:32:15
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-08-30 18:50:16
 */
#include <stdint.h>

#ifndef __PCL_TOOLS_H__
#define __PCL_TOOLS_H__
#include <omp.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_config.h>

#include "basis/data_type.h"
#include "basis/pcl_types.h"
namespace minco_local_planner::utils {

using namespace basis;

inline bool LoadPointCloud(const std::string &map_path,
                           PointCloud3d &cloud_out) {
  PointCloud3d cloud_map;
  auto flag = pcl::io::loadPCDFile(map_path, cloud_map);
  if (flag < 0) {
    return false;
  }
  cloud_map.width = cloud_map.points.size();
  cloud_map.height = 1;
  cloud_out = cloud_map;
  return true;
}

inline void GenerateGridMap(const PointCloud3d &cloud, Pose2d &origin,
                            Vec2i &dim, std::vector<int8_t> &data,
                            const double &resolution = 0.05,
                            const double &inf_size = 0.3) {
  double x_min{0}, x_max{0}, y_min{0}, y_max{0}, x{0}, y{0};
  // 得到点云地图最小的点

  const size_t points_size = cloud.points.size();
  for (size_t i = 0; i < points_size; i++) {
    if (i == 0) {
      x_min = x_max = cloud.points[i].x;
      y_min = y_max = cloud.points[i].y;
    }
    x = cloud.points[i].x;
    y = cloud.points[i].y;
    x_min = std::min(x, x_min);
    x_max = std::max(x, x_max);
    y_min = std::min(y, y_min);
    y_max = std::max(y, y_max);
  }
  // 需要进行再膨胀
  if (points_size > 0) {
    x_min = x_min - inf_size;
    y_min = y_min - inf_size;
    x_max = x_max + inf_size;
    y_max = y_max + inf_size;
  }

  // 得到栅格地图的起点
  origin = Vec3d(x_min, y_min, 0);
  // 得到栅格地图的尺寸
  dim[0] = ceil((x_max - x_min) / resolution);
  dim[1] = ceil((y_max - y_min) / resolution);
  // 创建栅格地图
  data.clear();
  data.resize(dim[0] * dim[1]);
  data.assign(dim[0] * dim[1], 0);
  // 膨胀地图
  int inf_step = ceil(inf_size / resolution);

  if (inf_step == 0) {
#pragma omp parallel for num_threads(32)

    for (size_t k = 0; k < points_size; k++) {
      const auto point = cloud.points[k];
      int i = (point.x - x_min) / resolution;
      if (i < 0 || i > dim[0] - 1) continue;
      int j = (point.y - y_min) / resolution;
      if (j < 0 || j > dim[1] - 1) continue;

      data[i + (j)*dim[0]] = 100;
    }

  } else {
#pragma omp parallel for num_threads(32)
    for (size_t k = 0; k < points_size; k++) {
      const auto point = cloud.points[k];
      int i = (point.x - x_min) / resolution;
      if (i < 0 || i > dim[0] - 1) continue;
      int j = (point.y - y_min) / resolution;
      if (j < 0 || j > dim[1] - 1) continue;
      for (int x = -inf_step; x < inf_step; ++x)
        for (int y = -inf_step; y < inf_step; ++y) {
          if (i + x < 0 || i + x > dim[0] - 1) continue;
          if (j + y < 0 || j + y > dim[1] - 1) continue;
          data[i + x + (j + y) * dim[0]] = 100;
        }
    }
  }
}

inline void downSampling(PointCloud3d &cloud, const double &res = 0.2) {
  PointCloud3d cloud_in = cloud;

  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud_in.makeShared());
  sor.setLeafSize((float)res, (float)res, (float)res);
  sor.filter(cloud);
}

inline void TransformPointCloud(PointCloud3d &cloud, const Pose2d &map_to_base,
                                const Pose2d &base_to_laser) {
  Aff3d map_to_base_tf = Aff3d::Identity();
  PointCloud3d cloud_out;
  map_to_base_tf.translation() << map_to_base[0], map_to_base[1], 0;
  map_to_base_tf.rotate(
      Eigen::AngleAxisd(map_to_base[2], Eigen::Vector3d::UnitZ()));
  Aff3d base_to_laser_tf = Aff3d::Identity();
  base_to_laser_tf.translation() << base_to_laser[0], base_to_laser[1], 0;
  base_to_laser_tf.rotate(
      Eigen::AngleAxisd(base_to_laser[2], Eigen::Vector3d::UnitZ()));
  Aff3d map_to_laser_tf = map_to_base_tf * base_to_laser_tf;

  pcl::transformPointCloud(cloud, cloud_out, map_to_laser_tf);
  cloud = cloud_out;
}

inline void TransformPointCloud(PointCloud3d &cloud, const Pose2d &pose) {
  Aff3d transform = Aff3d::Identity();
  PointCloud3d cloud_out;
  transform.translation() << pose[0], pose[1], 0;
  transform.rotate(Eigen::AngleAxisd(pose[2], Eigen::Vector3d::UnitZ()));
  pcl::transformPointCloud(cloud, cloud_out, transform);
  cloud = cloud_out;
}

inline void TransformPointCloud(const PointCloud3d &cloud_in,
                                const Pose2d &pose, PointCloud3d &cloud_out) {
  Aff3d transform = Aff3d::Identity();
  transform.translation() << pose[0], pose[1], 0;
  transform.rotate(Eigen::AngleAxisd(pose[2], Eigen::Vector3d::UnitZ()));
  pcl::transformPointCloud(cloud_in, cloud_out, transform);
}

inline void TransformPointCloud(const PointCloud3d &cloud_in, const Pose2d &dir,
                                const double &angle, PointCloud3d &cloud_out) {
  Aff3d transform = Aff3d::Identity();
  transform.rotate(Eigen::AngleAxisd(angle, dir));
  pcl::transformPointCloud(cloud_in, cloud_out, transform);
}

}  // namespace minco_local_planner::utils

#endif /* __PCL_TOOLS_H__ */
