/**
 * @Author: Xia Yunkai
 * @Date:   2023-08-24 19:56:08
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-08-24 20:22:11
 */
#include "raycast.h"

#include <Eigen/Eigen>
#include <cmath>
#include <iostream>

int signum(int x) { return x == 0 ? 0 : x < 0 ? -1 : 1; }

double mod(double value, double modulus) {
  return fmod(fmod(value, modulus) + modulus, modulus);
}

double intbound(double s, double ds) {
  // Find the smallest positive t such that s+t*ds is an integer.
  if (ds < 0) {
    return intbound(-s, -ds);
  } else {
    s = mod(s, 1);
    // problem is now s+t*ds = 1
    return (1 - s) / ds;
  }
}

bool RayCaster::setInput(const Eigen::Vector2d& start,
                         const Eigen::Vector2d& end) {
  start_ = start;
  end_ = end;
  // max_ = max;
  // min_ = min;

  x_ = (int)std::floor(start_.x());
  y_ = (int)std::floor(start_.y());
  // z_ = (int)std::floor(start_.z());
  endX_ = (int)std::floor(end_.x());
  endY_ = (int)std::floor(end_.y());
  // endZ_ = (int)std::floor(end_.z());
  direction_ = (end_ - start_);
  maxDist_ = direction_.squaredNorm();

  // Break out direction vector.
  dx_ = endX_ - x_;
  dy_ = endY_ - y_;
  // dz_ = endZ_ - z_;

  // Direction to increment x,y,z when stepping.
  stepX_ = (int)signum((int)dx_);
  stepY_ = (int)signum((int)dy_);
  // stepZ_ = (int)signum((int)dz_);

  // See description above. The initial values depend on the fractional
  // part of the origin.
  tMaxX_ = intbound(start_.x(), dx_);
  tMaxY_ = intbound(start_.y(), dy_);
  // tMaxZ_ = intbound(start_.z(), dz_);

  // The change in t when taking a step (always positive).
  tDeltaX_ = ((double)stepX_) / dx_;
  tDeltaY_ = ((double)stepY_) / dy_;
  // tDeltaZ_ = ((double)stepZ_) / dz_;

  dist_ = 0;

  step_num_ = 0;

  // Avoids an infinite loop.
  if (stepX_ == 0 && stepY_ == 0 /*&& stepZ_ == 0*/)
    return false;
  else
    return true;
}

bool RayCaster::step(Eigen::Vector2d& ray_pt) {
  // if (x_ >= min_.x() && x_ < max_.x() && y_ >= min_.y() && y_ < max_.y() &&
  // z_ >= min_.z() && z_ < max_.z())
  ray_pt = Eigen::Vector2d(x_, y_ /*, z_*/);

  // step_num_++;

  // dist_ = (Eigen::Vector3d(x_, y_, z_) - start_).squaredNorm();

  if (x_ == endX_ && y_ == endY_) {
    return false;
  }

  if (tMaxX_ < tMaxY_) {
    x_ += stepX_;
    tMaxX_ += tDeltaX_;

  } else {
    y_ += stepY_;
    tMaxY_ += tDeltaY_;
  }

  return true;
}
