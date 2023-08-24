/**
 * @Author: Yunkai Xia
 * @Date:   2023-07-20 09:00:48
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-08-24 17:39:48
 */
#include <stdint.h>

#ifndef __POSE2D_H__
#define __POSE2D_H__

#include "data_type.h"
#include "utils/math.h"
namespace minco_local_planner::sensors {
using namespace basis;
typedef Vec3d Pose2d;

// 带时间戳的位姿
struct Pose2dStamped {
  Pose2dStamped() : pose(Vec3d::Zero()), timestamp(0.0) {}
  Pose2dStamped(const double& x, const double& y, const double& yaw,
                const double& time)
      : pose(Vec3d(x, y, yaw)), timestamp(time) {}
  Pose2d pose;
  double timestamp;
};
//! Holds a pose 2d (x,y,th) with covariance (3x3)
struct Pose2dCov {
  //
  Pose2d pose;
  Mat3f cov;

  Pose2dCov() {
    pose.setZero();
    cov.setZero();
  }

  //! Initialize using the provided parameters.
  Pose2dCov(const double& x, const double& y, const double& t,
            const double& std_xx, const double& std_xy, const double& std_xt,
            const double& std_yy, const double& std_yt, const double& std_tt) {
    pose[0] = x;
    pose[1] = y;
    pose[2] = t;
    cov(0, 0) = std_xx;
    cov(1, 0) = cov(0, 1) = std_xy;
    cov(2, 0) = cov(0, 2) = std_xt;
    cov(1, 1) = std_yy;
    cov(1, 2) = cov(2, 1) = std_yt;
    cov(2, 2) = std_tt;
  }

  friend std::ostream& operator<<(std::ostream& os, const Pose2dCov& obj) {
    os << "\n[mean] : \n" << obj.pose;
    os << "\n[cov ] : \n" << obj.cov;
    return os;
  }
};

// 位姿增量式增加
inline Pose2d addPose2d(const Pose2d& origin, const Pose2d& incr) {
  Vec3d ret;
  ret[0] = origin[0] + incr[0] * cos(origin[2]) - incr[1] * sin(origin[2]);
  ret[1] = origin[1] + incr[0] * sin(origin[2]) + incr[1] * cos(origin[2]);
  ret[2] = normalizeAngleRad(origin[2] + incr[2]);
  return ret;
}

// 两个位姿间的相对位置
inline Pose2d subPose2d(const Pose2d& origin, const Pose2d& pose) {
  Vec3d ret;
  const auto cos_ = std::cos(origin[2]);
  const auto sin_ = std::sin(origin[2]);
  ret[0] = (pose[0] - origin[0]) * cos_ + (pose[1] - origin[1]) * sin_;
  ret[1] = -(pose[0] - origin[0]) * sin_ + (pose[1] - origin[1]) * cos_;
  ret[2] = normalizeAngleRad(pose[2] - origin[2]);
  return ret;
}

inline Pose2dCov addPose2dCov(const Pose2dCov& origin, const Pose2dCov& incr) {
  Pose2dCov ret;
  ret.pose = addPose2d(origin.pose, incr.pose);
  Mat3f J_1;
  Mat3f J_2;
  const auto c = std::cos(origin.pose[2]);
  const auto s = std::sin(origin.pose[2]);
  J_1 << 1, 0, -s * incr.pose[0] - c * incr.pose[1], 0, 1,
      c * incr.pose[0] - s * incr.pose[1], 0, 0, 1;

  J_2 << c, -s, 0, s, c, 0, 0, 0, 1;

  ret.cov =
      J_1 * origin.cov * J_1.transpose() + J_2 * incr.cov * J_2.transpose();
  return ret;
}

inline double getDist(const Pose2d& incr) {
  return sqrt(std::pow(incr[0], 2) + std::pow(incr[1], 2));
}

inline double getDistBetween(const Pose2d& pose1, const Pose2d& pose2) {
  Pose2d tmp = pose1 - pose2;
  return getDist(tmp);
}

inline double getAngularNormDist(const Pose2d& pose1, const Pose2d& pose2) {
  return normalizeAngleRad(pose1[2] - pose2[2]);
}

inline Vec2f getPosition(const Pose2d& pose) { return Vec2f(pose(0), pose(1)); }

inline double getHeading(const Pose2d& pose) { return pose[2]; }

inline Pose2d getBaseOffsetPose(const Pose2d& pose, const double& offset) {
  Pose2d opose(0, offset, 0);
  return addPose2d(pose, opose);
}

inline bool addStepPose2d(const Pose2d& p1, const Pose2d& p2,
                          const double& maxDist, const double& maxRotation) {
  if (getDistBetween(p1, p2) > maxDist) return true;
  if (fabs(getAngularNormDist(p1, p2)) > maxRotation) return true;
  return false;
}

inline Vec2f getBaseOffset(const Pose2d& pose, const double& offset) {
  return getPosition(getBaseOffsetPose(pose, offset));
}

inline double getDirectionIncr(const Pose2d& p) {
  if (p(0) >= 0)  // X is forward.
    return 1.;
  return -1;
}

inline double getDirection(const Pose2d& p1, const Pose2d& p2) {
  Pose2d dir = subPose2d(p1, p2);
  return getDirectionIncr(dir);
}

inline bool forwardDirection(const Pose2d& p1, const Pose2d& p2) {
  if (getDirection(p1, p2) > 0) return true;
  return false;
}

inline double getAtan2(const Pose2d& p1, const Pose2d& p2) {
  const auto delt_x = p1(0) - p2(0);
  const auto delt_y = p1(1) - p2(1);
  return std::atan2(delt_y, delt_x);
}

inline Quaterniond getQuaterion(const Pose2d& p) {
  double sy = std::sin(p[2] * 0.5);
  double cy = std::cos(p[2] * 0.5);
  double sp = 0.;
  double cp = 1.;
  double sr = 0.;
  double cr = 1.;
  double w = cr * cp * cy + sr * sp * sy;
  double x = sr * cp * cy - cr * sp * sy;
  double y = cr * sp * cy + sr * cp * sy;
  double z = cr * cp * sy - sr * sp * cy;
  return Quaterniond(w, x, y, z);
}

inline bool validPose2dDiff(const Pose2d& p1, const Pose2d& p2,
                            const double& maxDist,
                            const double& maxAngularDist) {
  if (getDistBetween(p1, p2) > maxDist) {
    return false;
  }
  if (fabs(getAngularNormDist(p1, p2)) > maxAngularDist) {
    return false;
  }
  return true;
}

inline Eigen::MatrixXd pseudoInverse(Eigen::MatrixXd& mat) {
  // Calculate using the SVD.
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      mat, Eigen::ComputeThinU | Eigen::ComputeThinV);
  // svd.matrixU();
  // svd.matrixV();
  Eigen::MatrixXd tmp = svd.singularValues();

  int size = mat.rows();
  if (size > mat.cols()) size = mat.cols();
  for (int i = 0; i < size; i++)
    if (tmp(i, i) < std::numeric_limits<double>::epsilon())
      tmp(i, i) = 0.;
    else
      tmp(i, i) = 1. / tmp(i, i);
  return svd.matrixV().transpose() * tmp.transpose() *
         svd.matrixU().transpose();
}

inline Pose2d pose2dFromAffine3d(const Aff3f& T) {
  Pose2d ret;
  ret[0] = T.translation()[0];
  ret[1] = T.translation()[1];
  ret[2] = T.rotation().eulerAngles(0, 1, 2)[2];
  return ret;
}

inline Mat3f cov6toCov3(const Mat6f& cov6) {
  Mat3f cov3;
  cov3(0, 0) = cov6(0, 0);
  cov3(1, 1) = cov6(1, 1);
  cov3(0, 1) = cov6(0, 1);
  cov3(1, 0) = cov6(1, 0);
  cov3(2, 2) = cov6(5, 5);
  cov3(0, 2) = cov6(0, 5);
  cov3(1, 2) = cov6(1, 5);
  cov3(2, 0) = cov6(5, 0);
  cov3(2, 1) = cov6(5, 1);
  return cov3;
}

inline void pose2dClearDependence(Pose2dCov& posecov) {
  posecov.cov(0, 1) = 0.;
  posecov.cov(1, 0) = 0.;
  posecov.cov(0, 2) = 0.;
  posecov.cov(1, 2) = 0.;
  posecov.cov(2, 0) = 0.;
  posecov.cov(2, 1) = 0.;
}

}  // namespace minco_local_planner::sensors

#endif /* __POSE2D_H__ */
