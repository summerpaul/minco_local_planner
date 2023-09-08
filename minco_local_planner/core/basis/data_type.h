/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-24 14:19:40
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-09-08 14:54:28
 */
#include <stdint.h>

#ifndef __DATA_TYPE_H__
#define __DATA_TYPE_H__
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <cmath>
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
typedef Vecd<Eigen::Dynamic> VecXd;

template <int N>
using Veci = Eigen::Matrix<int, N, 1>;

typedef Veci<2> Vec2i;
typedef Veci<3> Vec3i;
typedef Veci<Eigen::Dynamic> VecXi;

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

typedef Matd<Eigen::Dynamic, Eigen::Dynamic> MatDDd;

template <int N>
using MatDNd = Eigen::Matrix<double, Eigen::Dynamic, N>;

template <int N>
using MatNDd = Eigen::Matrix<double, N, Eigen::Dynamic>;

typedef Eigen::Transform<double, 2, Eigen::Affine> Aff2d;

typedef Eigen::Transform<double, 3, Eigen::Affine> Aff3d;

typedef std::vector<double> Vec_d;

typedef Eigen::Quaternion<double> Quatd;

typedef vec_Vec2d Points2d;
typedef vec_Vec2d Path2d;
typedef Vec3d Pose2d;

template <int N>
using vec_Vecd = vec_E<Vecd<N>>;

const double kBigEPS = 1e-1;

const double kEPS = 1e-6;

const double kSmallEPS = 1e-10;

const double kPi = std::acos(-1.0);

const double kInf = 1e20;

}  // namespace minco_local_planner::basis
#endif /* __DATA_TYPE_H__ */
