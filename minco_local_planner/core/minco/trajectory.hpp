/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to
   deal in the Software without restriction, including without limitation the
   rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
   sell copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
   FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
   IN THE SOFTWARE.
*/

#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP

#include <cfloat>
#include <cmath>
#include <iostream>
#include <vector>

#include "basis/data_type.h"
#include "root_finder.hpp"

namespace minco_local_planner::minco {

using namespace basis;
constexpr double PI = 3.1415926;
typedef Matd<2, 6> CoefficientMat;
typedef Matd<2, 5> VelCoefficientMat;
typedef Matd<2, 4> AccCoefficientMat;

class Piece  // component from poly
{
 private:  // duration + coeffMat
  double duration;
  CoefficientMat coeffMat;

  //@yuwei
  int dim = 2;
  int order = 5;
  //@yuwei: filter the Singularity (enable forward and reverse moving)
  int singul;
  // common::VehicleParam vp_;

 public:
  Piece() = default;

  Piece(double dur, const CoefficientMat &cMat, int s)
      : duration(dur), coeffMat(cMat), singul(s) {}
  //@yuwei
  inline int getDim() const { return dim; }
  //@yuwei
  inline int getOrder() const { return order; }

  inline double getDuration() const { return duration; }

  inline const CoefficientMat &getCoeffMat() const { return coeffMat; }

  inline VelCoefficientMat getVelCoeffMat() const {
    VelCoefficientMat velCoeffMat;
    int n = 1;
    for (int i = 4; i >= 0; i--) {
      velCoeffMat.col(i) = n * coeffMat.col(i);
      n++;
    }
    return velCoeffMat;
  }

  inline int getSingul(const double &t) const { return singul; }

  // the point in the rear axle center
  inline Eigen::Vector2d getPos(const double &t) const {
    Eigen::Vector2d pos(0.0, 0.0);
    double tn = 1.0;
    for (int i = order; i >= 0; i--) {
      pos += tn * coeffMat.col(i);
      tn *= t;
    }
    return pos;
  }

  inline Eigen::Matrix2d getR(const double &t) const {
    Eigen::Vector2d current_v = getdSigma(t);
    Eigen::Matrix2d rotation_matrix;
    rotation_matrix << current_v(0), -current_v(1), current_v(1), current_v(0);
    rotation_matrix = singul * rotation_matrix / current_v.norm();

    return rotation_matrix;
  }

  inline Eigen::Matrix2d getRdot(const double &t) const {
    Eigen::Vector2d current_v = getdSigma(t);
    Eigen::Vector2d current_a = getddSigma(t);
    Eigen::Matrix2d temp_a_ba, temp_v_bv;
    temp_a_ba << current_a(0), -current_a(1), current_a(1), current_a(0);
    temp_v_bv << current_v(0), -current_v(1), current_v(1), current_v(0);
    Eigen::Matrix2d R_dot = singul * (temp_a_ba / current_v.norm() -
                                      temp_v_bv / pow(current_v.norm(), 3) *
                                          (current_v.transpose() * current_a));

    return R_dot;
  }

  // the point in the rear axle center
  inline std::vector<Eigen::Vector2d> setBoundObs(const double &t, double d_x,
                                                  double d_y) const {
    std::vector<Eigen::Vector2d> BoundObs;
    Eigen::Vector2d pos = getPos(t);
    double angle = getAngle(t);

    double cos_theta = cos(angle);
    double sin_theta = sin(angle);
    double c_x = pos(0);
    double c_y = pos(1);

    double d_wx = d_y * sin_theta;
    double d_wy = d_y * cos_theta;
    double d_lx = d_x * cos_theta;
    double d_ly = d_x * sin_theta;
    // Counterclockwise from left-front vertex
    BoundObs.push_back(Eigen::Vector2d(c_x - d_wx + d_lx, c_y + d_wy + d_ly));
    BoundObs.push_back(Eigen::Vector2d(c_x - d_wx - d_lx, c_y - d_ly + d_wy));
    BoundObs.push_back(Eigen::Vector2d(c_x + d_wx - d_lx, c_y - d_wy - d_ly));
    BoundObs.push_back(Eigen::Vector2d(c_x + d_wx + d_lx, c_y + d_ly - d_wy));

    return BoundObs;
  }

  inline Eigen::Vector2d getdSigma(const double &t) const {
    Eigen::Vector2d dsigma(0.0, 0.0);
    double tn = 1.0;
    int n = 1;

    for (int i = order - 1; i >= 0; i--) {
      dsigma += n * tn * coeffMat.col(i);
      tn *= t;
      n++;
    }
    return dsigma;
  }

  inline Eigen::Vector2d getddSigma(const double &t) const {
    Eigen::Vector2d ddsigma(0.0, 0.0);
    double tn = 1.0;
    int m = 1;
    int n = 2;

    for (int i = order - 2; i >= 0; i--) {
      ddsigma += m * n * tn * coeffMat.col(i);
      tn *= t;
      m++;
      n++;
    }
    return ddsigma;
  }

  inline Eigen::Vector2d getdddSigma(const double &t) const {
    Eigen::Vector2d dddsigma(0.0, 0.0);
    double tn = 1.0;
    int l = 1;
    int m = 2;
    int n = 3;

    std::cout << "coeffMat is" << coeffMat << std::endl;
    for (int i = order - 3; i >= 0; i--) {
      dddsigma += l * m * n * tn * coeffMat.col(i);

      tn *= t;
      l++;
      m++;
      n++;
    }
    std::cout << "current time is" << t << std::endl;
    std::cout << "dddsigma is" << dddsigma << std::endl;
    return dddsigma;
  }

  //@yuwei get heading angle theta
  inline double getAngle(const double &t) const {
    Eigen::Vector2d dsigma = getdSigma(t);

    return std::atan2(singul * dsigma(1), singul * dsigma(0));  //[-PI, PI]
  }

  //@yuwei
  inline double getCurv(const double &t) const {
    Eigen::Vector2d dsigma = getdSigma(t);
    Eigen::Vector2d ddsigma = getddSigma(t);

    if (dsigma.norm() < 1e-6) {
      return 0.0;
    } else {
      //[-PI/2, PI/2]
      //   std::cout<<"numerator: "<<
      //   (dsigma(0)*ddsigma(1)-dsigma(1)*ddsigma(0))<<" deno:
      //   "<<std::pow(dsigma.norm(), 3)
      //   <<" cur: "<<(dsigma(0)*ddsigma(1)-dsigma(1)*ddsigma(0))/
      //   sqrt(std::pow((std::pow(dsigma.norm(), 2)+1e-4),3))<<"\n";
      // std::cout<<"cur: "<<(dsigma(0)*ddsigma(1)-dsigma(1)*ddsigma(0))/
      // std::pow(dsigma.norm(), 3)<<"vel: "
      // <<dsigma.norm()<<" acc: "<< ddsigma.norm()<<std::endl;
      std::cout << "cur: "
                << (dsigma(0) * ddsigma(1) - dsigma(1) * ddsigma(0)) /
                       std::pow(dsigma.norm(), 3)
                // <<" numera: "<<(dsigma(0)*ddsigma(1)-dsigma(1)*ddsigma(0))<<"
                // deno: "<<std::pow(dsigma.norm(), 3)<< "vel:
                // "<<dsigma.transpose()<<"acc: "<<ddsigma.transpose()
                << "\n";
      return singul * (dsigma(0) * ddsigma(1) - dsigma(1) * ddsigma(0)) /
             std::pow(dsigma.norm(), 3);
    }
  }

  //@yuwei
  inline double getVel(const double &t) const {
    Eigen::Vector2d dsigma = getdSigma(t);

    return singul * dsigma.norm();
  }

  //@yuwei
  inline double getAcc(const double &t) const {
    Eigen::Vector2d dsigma = getdSigma(t);
    Eigen::Vector2d ddsigma = getddSigma(t);

    if (dsigma.norm() < 1e-6) {
      return 0.0;
    } else {
      return singul * (dsigma(0) * ddsigma(0) + dsigma(1) * ddsigma(1)) /
             dsigma.norm();
    }
  }

  inline CoefficientMat normalizePosCoeffMat() const {
    CoefficientMat nPosCoeffsMat;
    double t = 1.0;
    for (int i = order; i >= 0; i--) {
      nPosCoeffsMat.col(i) = coeffMat.col(i) * t;
      t *= duration;
    }
    return nPosCoeffsMat;
  }
};

class Trajectory {
 private:
  typedef std::vector<Piece> Pieces;
  Pieces pieces;
  int direction;

 public:
  Trajectory() = default;

  Trajectory(const std::vector<double> &durs,
             const std::vector<CoefficientMat> &cMats, int s) {
    int N = std::min(durs.size(), cMats.size());
    pieces.reserve(N);
    for (int i = 0; i < N; i++) {
      pieces.emplace_back(durs[i], cMats[i], s);
    }
    direction = s;
  }
  inline int getSingul(double t) {
    double inner_t = t;
    if (t > getTotalDuration()) {
      inner_t = getTotalDuration();
    }
    int pieceIdx = locatePieceIdx(inner_t);
    return pieces[pieceIdx].getSingul(inner_t);
  }

  inline int getDirection() const { return direction; }

  inline int getPieceNum() const { return pieces.size(); }

  inline Eigen::VectorXd getDurations() const {
    int N = getPieceNum();
    Eigen::VectorXd durations(N);
    for (int i = 0; i < N; i++) {
      durations(i) = pieces[i].getDuration();
    }
    return durations;
  }

  inline double getTotalDuration() const {
    int N = getPieceNum();
    double totalDuration = 0.0;
    for (int i = 0; i < N; i++) {
      totalDuration += pieces[i].getDuration();
    }
    return totalDuration;
  }

  //@yuwei: revise to 2d case
  inline Eigen::MatrixXd getPositions() const {
    int N = getPieceNum();
    Eigen::MatrixXd positions(2, N + 1);
    for (int i = 0; i < N; i++) {
      positions.col(i) = pieces[i].getCoeffMat().col(5);
    }
    positions.col(N) = pieces[N - 1].getPos(pieces[N - 1].getDuration());
    return positions;
  }

  inline const Piece &operator[](int i) const { return pieces[i]; }

  inline Piece &operator[](int i) { return pieces[i]; }

  inline void clear(void) {
    pieces.clear();
    return;
  }

  inline Pieces::const_iterator begin() const { return pieces.begin(); }

  inline Pieces::const_iterator end() const { return pieces.end(); }

  inline Pieces::iterator begin() { return pieces.begin(); }

  inline Pieces::iterator end() { return pieces.end(); }

  inline void reserve(const int &n) {
    pieces.reserve(n);
    return;
  }

  inline void emplace_back(const Piece &piece) {
    pieces.emplace_back(piece);
    return;
  }

  inline void emplace_back(const double &dur, const CoefficientMat &cMat,
                           int s) {
    pieces.emplace_back(dur, cMat, s);
    direction = s;
    return;
  }

  inline void append(const Trajectory &traj) {
    pieces.insert(pieces.end(), traj.begin(), traj.end());
    return;
  }

  inline int locatePieceIdx(double &t) const {
    int N = getPieceNum();
    int idx;
    double dur;
    for (idx = 0; idx < N && t > (dur = pieces[idx].getDuration()); idx++) {
      t -= dur;
    }
    if (idx == N) {
      idx--;
      t += pieces[idx].getDuration();
    }
    return idx;
  }

  inline int locatePieceIdx2(double t) const {
    int N = getPieceNum();
    std::cout << "N = getPieceNum() is" << N << std::endl;
    int idx;
    double dur;
    for (idx = 0; idx < N && t > (dur = pieces[idx].getDuration()); idx++) {
      t -= dur;
    }
    if (idx == N) {
      idx--;
      t += pieces[idx].getDuration();
    }
    return idx;
  }

  //@yuwei
  inline Eigen::Vector2d getPos(double t) const {
    int pieceIdx = locatePieceIdx(t);
    return pieces[pieceIdx].getPos(t);
  }

  // MCJ
  inline Eigen::Matrix2d getR(double t) const {
    int pieceIdx = locatePieceIdx(t);
    return pieces[pieceIdx].getR(t);
  }

  // MCJ
  inline Eigen::Matrix2d getRdot(double t) const {
    int pieceIdx = locatePieceIdx(t);
    return pieces[pieceIdx].getRdot(t);
  }

  //@yuwei
  // inline std::vector<Eigen::Vector2d> getBoundPts(double t) const
  // {
  //     int pieceIdx = locatePieceIdx(t);
  //     return pieces[pieceIdx].getBoundPts(t);
  // }

  //@yuwei
  inline std::vector<Eigen::Vector2d> setBoundObs(double t, double d_x,
                                                  double d_y) const {
    int pieceIdx = locatePieceIdx(t);
    return pieces[pieceIdx].setBoundObs(t, d_x, d_y);
  }

  //@yuwei
  inline Eigen::Vector2d getdSigma(double t) const {
    int pieceIdx = locatePieceIdx(t);
    return pieces[pieceIdx].getdSigma(t);
  }

  //@yuwei
  inline Eigen::Vector2d getddSigma(double t) const {
    int pieceIdx = locatePieceIdx(t);
    // std::cout << "pieceIdx is"<< pieceIdx << std::endl;
    // std::cout << "t is"<< t << std::endl;
    return pieces[pieceIdx].getddSigma(t);
  }

  //@yuwei
  inline double getVel(double t) const {
    int pieceIdx = locatePieceIdx(t);
    return pieces[pieceIdx].getVel(t);
  }

  //@yuwei
  inline double getAcc(double t) const {
    int pieceIdx = locatePieceIdx(t);
    return pieces[pieceIdx].getAcc(t);
  }

  //@yuwei
  inline double getAngle(double t) const {
    int pieceIdx = locatePieceIdx(t);
    return pieces[pieceIdx].getAngle(t);
  }

  //@yuwei
  inline double getCurv(double t) const {
    int pieceIdx = locatePieceIdx(t);
    return pieces[pieceIdx].getCurv(t);
  }

  //@yuwei
  // inline double getSteer(double t) const
  // {
  //     int pieceIdx = locatePieceIdx(t);
  //     return pieces[pieceIdx].getSteer(t);
  // }

  //@yuwei
  inline Eigen::Vector2d getJuncPos(int juncIdx) const {
    if (juncIdx != getPieceNum()) {
      return pieces[juncIdx].getCoeffMat().col(5);
    } else {
      return pieces[juncIdx - 1].getPos(pieces[juncIdx - 1].getDuration());
    }
  }

  //@yuwei
  inline Eigen::Vector2d getJuncdSigma(int juncIdx) const {
    if (juncIdx != getPieceNum()) {
      // std::cout <<"[getJuncdSigma] get coeff" <<
      // pieces[juncIdx].getCoeffMat() << std::endl;
      return pieces[juncIdx].getCoeffMat().col(4);
    } else {
      return pieces[juncIdx - 1].getdSigma(pieces[juncIdx - 1].getDuration());
    }
  }

  //@yuwei
  inline Eigen::Vector2d getJuncddSigma(int juncIdx) const {
    // std::cout <<"[getJuncdSigma] get coeff" << pieces[juncIdx].getCoeffMat()
    // << std::endl;
    if (juncIdx != getPieceNum()) {
      return pieces[juncIdx].getCoeffMat().col(3) * 2.0;
    } else {
      return pieces[juncIdx - 1].getddSigma(pieces[juncIdx - 1].getDuration());
    }
  }

  // zxzx
  inline std::pair<int, double> locatePieceIdxWithRatio(double &t) const {
    int N = getPieceNum();
    int idx;
    double dur;
    for (idx = 0; idx < N && t > (dur = pieces[idx].getDuration()); idx++) {
      t -= dur;
    }
    if (idx == N) {
      idx--;
      t += pieces[idx].getDuration();
    }
    std::pair<int, double> idx_ratio;
    idx_ratio.first = idx;
    idx_ratio.second = t / dur;
    return idx_ratio;
  }

  // zxzx
  inline Eigen::Vector2d getPoswithIdxRatio(
      double t, std::pair<int, double> &idx_ratio) const {
    idx_ratio = locatePieceIdxWithRatio(t);
    return pieces[idx_ratio.first].getPos(t);
  }
};

}  // namespace minco_local_planner::minco
#endif