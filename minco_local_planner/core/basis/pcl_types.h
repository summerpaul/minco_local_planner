/**
 * @Author: Xia Yunkai
 * @Date:   2023-08-27 22:53:15
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-08-27 22:54:15
 */
#include <stdint.h>

#ifndef __PCL_TYPES_H__
#define __PCL_TYPES_H__
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
namespace minco_local_planner::basis {
using PointCloud3d = pcl::PointCloud<pcl::PointXYZ>;
using PointCloud3di = pcl::PointCloud<pcl::PointXYZI>;
using PointCloud2d = pcl::PointCloud<pcl::PointXY>;
}  // namespace minco_local_planner::basis

#endif /* __PCL_TYPES_H__ */
