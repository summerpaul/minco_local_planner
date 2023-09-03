/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-24 14:19:40
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-09-03 09:57:16
 */
#include <stdint.h>

#ifndef __DATA_TYPE_H__
#define __DATA_TYPE_H__
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <iostream>
#include <vector>
namespace minco_local_planner::basis {

template <int N>
using Vecd = Eigen::Matrix<double, N, 1>;

typedef Vecd<2> Vec2d;
typedef Vecd<3> Vec3d;
typedef Vecd<4> Vec4d;
typedef Vecd<5> Vec5d;
typedef Vecd<6> Vec6d;

template <int N>
using Veci = Eigen::Matrix<int, N, 1>;

typedef Veci<2> Vec2i;
typedef Veci<3> Vec3i;

template <typename T>
using vec_E = std::vector<T, Eigen::aligned_allocator<T>>;

typedef vec_E<Vec2d> vec_Vec2d;

typedef vec_E<Vec2i> vec_Vec2i;

typedef vec_E<Vec3d> vec_Vec3d;

typedef vec_E<Vec3i> vec_Vec3i;

template <int M, int N>
using Matd = Eigen::Matrix<double, M, N>;

typedef Matd<2, 2> Mat2d;

typedef Matd<3, 3> Mat3d;

typedef Matd<4, 4> Mat4d;

typedef Matd<6, 6> Mat6d;

typedef Matd<4, 2> Mat4x2d;

typedef Matd<Eigen::Dynamic, Eigen::Dynamic> MatDd;

typedef Eigen::Transform<double, 2, Eigen::Affine> Aff2d;

typedef Eigen::Transform<double, 3, Eigen::Affine> Aff3d;

typedef std::vector<double> Vec_d;

typedef Eigen::Quaternion<double> Quatd;

typedef vec_Vec2d Points2d;
typedef vec_Vec2d Path2d;
typedef Vec3d Pose2d;



}  // namespace minco_local_planner::basis
#endif /* __DATA_TYPE_H__ */


///Set red font in printf funtion
#ifndef ANSI_COLOR_RED
#define ANSI_COLOR_RED "\x1b[1;31m"
#endif
///Set green font in printf funtion
#ifndef ANSI_COLOR_GREEN
#define ANSI_COLOR_GREEN "\x1b[1;32m"
#endif
///Set yellow font in printf funtion
#ifndef ANSI_COLOR_YELLOW
#define ANSI_COLOR_YELLOW "\x1b[1;33m"
#endif
///Set blue font in printf funtion
#ifndef ANSI_COLOR_BLUE
#define ANSI_COLOR_BLUE "\x1b[1;34m"
#endif
///Set magenta font in printf funtion
#ifndef ANSI_COLOR_MAGENTA
#define ANSI_COLOR_MAGENTA "\x1b[1;35m"
#endif
///Set cyan font in printf funtion
#ifndef ANSI_COLOR_CYAN
#define ANSI_COLOR_CYAN "\x1b[1;36m"
#endif
///Reset font color in printf funtion
#ifndef ANSI_COLOR_RESET
#define ANSI_COLOR_RESET "\x1b[0m"
#endif
