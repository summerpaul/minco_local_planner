/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-24 14:19:40
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-08-24 20:54:30
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

typedef Matd<Eigen::Dynamic, Eigen::Dynamic> MatDf;

typedef Eigen::Transform<double, 2, Eigen::Affine> Aff2f;

typedef Eigen::Transform<double, 3, Eigen::Affine> Aff3f;

typedef std::vector<double> Vec_d;

typedef Eigen::Quaterniond Quaterniond;

typedef vec_Vec2d Points2d;
typedef vec_Vec2d Path2d;




}  // namespace minco_local_planner::basis
#endif /* __DATA_TYPE_H__ */
