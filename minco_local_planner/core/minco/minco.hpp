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

#ifndef MINCO_HPP
#define MINCO_HPP

#include <cmath>
#include <vector>

#include "trajectory.hpp"

namespace minco_local_planner::minco {

// The banded system class is used for solving
// banded linear system Ax=b efficiently.
// A is an N*N band matrix with lower band width lowerBw
// and upper band width upperBw.
// Banded LU factorization has O(N) time complexity.
class BandedSystem {
 public:
  // The size of A, as well as the lower/upper
  // banded width p/q are needed
  inline void create(const int &n, const int &p, const int &q) {
    // In case of re-creating before destroying
    destroy();
    N = n;
    lowerBw = p;
    upperBw = q;
    int actualSize = N * (lowerBw + upperBw + 1);
    ptrData = new double[actualSize];
    std::fill_n(ptrData, actualSize, 0.0);
    return;
  }

  inline void destroy() {
    if (ptrData != nullptr) {
      delete[] ptrData;
      ptrData = nullptr;
    }
    return;
  }

 private:
  int N;
  int lowerBw;
  int upperBw;
  // Compulsory nullptr initialization here
  double *ptrData = nullptr;

 public:
  // Reset the matrix to zero
  inline void reset(void) {
    std::fill_n(ptrData, N * (lowerBw + upperBw + 1), 0.0);
    return;
  }

  // The band matrix is stored as suggested in "Matrix Computation"
  inline const double &operator()(const int &i, const int &j) const {
    return ptrData[(i - j + upperBw) * N + j];
  }

  inline double &operator()(const int &i, const int &j) {
    return ptrData[(i - j + upperBw) * N + j];
  }

  // This function conducts banded LU factorization in place
  // Note that NO PIVOT is applied on the matrix "A" for efficiency!!!
  inline void factorizeLU() {
    int iM, jM;
    double cVl;
    for (int k = 0; k <= N - 2; k++) {
      iM = std::min(k + lowerBw, N - 1);
      cVl = operator()(k, k);
      for (int i = k + 1; i <= iM; i++) {
        if (operator()(i, k) != 0.0) {
          operator()(i, k) /= cVl;
        }
      }
      jM = std::min(k + upperBw, N - 1);
      for (int j = k + 1; j <= jM; j++) {
        cVl = operator()(k, j);
        if (cVl != 0.0) {
          for (int i = k + 1; i <= iM; i++) {
            if (operator()(i, k) != 0.0) {
              operator()(i, j) -= operator()(i, k) * cVl;
            }
          }
        }
      }
    }
    return;
  }

  // This function solves Ax=b, then stores x in b
  // The input b is required to be N*m, i.e.,
  // m vectors to be solved.
  template <typename EIGENMAT>
  inline void solve(EIGENMAT &b) const {
    int iM;
    for (int j = 0; j <= N - 1; j++) {
      iM = std::min(j + lowerBw, N - 1);
      for (int i = j + 1; i <= iM; i++) {
        if (operator()(i, j) != 0.0) {
          b.row(i) -= operator()(i, j) * b.row(j);
        }
      }
    }
    for (int j = N - 1; j >= 0; j--) {
      b.row(j) /= operator()(j, j);
      iM = std::max(0, j - upperBw);
      for (int i = iM; i <= j - 1; i++) {
        if (operator()(i, j) != 0.0) {
          b.row(i) -= operator()(i, j) * b.row(j);
        }
      }
    }
    return;
  }

  // This function solves ATx=b, then stores x in b
  // The input b is required to be N*m, i.e.,
  // m vectors to be solved.
  template <typename EIGENMAT>
  inline void solveAdj(EIGENMAT &b) const {
    int iM;
    for (int j = 0; j <= N - 1; j++) {
      b.row(j) /= operator()(j, j);
      iM = std::min(j + upperBw, N - 1);
      for (int i = j + 1; i <= iM; i++) {
        if (operator()(j, i) != 0.0) {
          b.row(i) -= operator()(j, i) * b.row(j);
        }
      }
    }
    for (int j = N - 1; j >= 0; j--) {
      iM = std::max(0, j - lowerBw);
      for (int i = iM; i <= j - 1; i++) {
        if (operator()(j, i) != 0.0) {
          b.row(i) -= operator()(j, i) * b.row(j);
        }
      }
    }
    return;
  }
};

class MinJerkOpt {
 public:
  MinJerkOpt() = default;
  ~MinJerkOpt() { A.destroy(); }

 private:
  int N;                    // pieceNum
  Eigen::MatrixXd headPVA;  // 2,3
  Eigen::MatrixXd tailPVA;  // 2,3

  Eigen::VectorXd T1;
  BandedSystem A;     // 6 * N, 6, 6
  Eigen::MatrixXd b;  // 6*N *2

  // Temp variables
  Eigen::VectorXd T2;
  Eigen::VectorXd T3;
  Eigen::VectorXd T4;
  Eigen::VectorXd T5;
  Eigen::MatrixXd gdC;
  double vmax_, amax_, kmax_;

 private:
  template <typename EIGENVEC>
  inline void addGradJbyT(EIGENVEC &gdT) const {
    for (int i = 0; i < N; i++) {
      gdT(i) += 36.0 * b.row(6 * i + 3).squaredNorm() +
                288.0 * b.row(6 * i + 4).dot(b.row(6 * i + 3)) * T1(i) +
                576.0 * b.row(6 * i + 4).squaredNorm() * T2(i) +
                720.0 * b.row(6 * i + 5).dot(b.row(6 * i + 3)) * T2(i) +
                2880.0 * b.row(6 * i + 5).dot(b.row(6 * i + 4)) * T3(i) +
                3600.0 * b.row(6 * i + 5).squaredNorm() * T4(i);
    }
    return;
  }

  template <typename EIGENMAT>
  inline void addGradJbyC(EIGENMAT &gdC) const {
    for (int i = 0; i < N; i++) {
      gdC.row(6 * i + 5) += 240.0 * b.row(6 * i + 3) * T3(i) +
                            720.0 * b.row(6 * i + 4) * T4(i) +
                            1440.0 * b.row(6 * i + 5) * T5(i);
      gdC.row(6 * i + 4) += 144.0 * b.row(6 * i + 3) * T2(i) +
                            384.0 * b.row(6 * i + 4) * T3(i) +
                            720.0 * b.row(6 * i + 5) * T4(i);
      gdC.row(6 * i + 3) += 72.0 * b.row(6 * i + 3) * T1(i) +
                            144.0 * b.row(6 * i + 4) * T2(i) +
                            240.0 * b.row(6 * i + 5) * T3(i);
    }

    return;
  }

  inline void solveAdjGradC(Eigen::MatrixXd &gdC) const {
    A.solveAdj(gdC);
    return;
  }
  //@yuwei revise to 2d
  template <typename EIGENVEC>
  inline void addPropCtoT(const Eigen::MatrixXd &adjGdC, EIGENVEC &gdT) const {
    Eigen::MatrixXd B1(6, 2), B2(3, 2);

    Eigen::RowVector2d negVel, negAcc, negJer, negSnp, negCrk;

    for (int i = 0; i < N - 1; i++) {
      negVel =
          -(b.row(i * 6 + 1) + 2.0 * T1(i) * b.row(i * 6 + 2) +
            3.0 * T2(i) * b.row(i * 6 + 3) + 4.0 * T3(i) * b.row(i * 6 + 4) +
            5.0 * T4(i) * b.row(i * 6 + 5));
      negAcc =
          -(2.0 * b.row(i * 6 + 2) + 6.0 * T1(i) * b.row(i * 6 + 3) +
            12.0 * T2(i) * b.row(i * 6 + 4) + 20.0 * T3(i) * b.row(i * 6 + 5));
      negJer = -(6.0 * b.row(i * 6 + 3) + 24.0 * T1(i) * b.row(i * 6 + 4) +
                 60.0 * T2(i) * b.row(i * 6 + 5));
      negSnp = -(24.0 * b.row(i * 6 + 4) + 120.0 * T1(i) * b.row(i * 6 + 5));
      negCrk = -120.0 * b.row(i * 6 + 5);

      B1 << negSnp, negCrk, negVel, negVel, negAcc, negJer;

      gdT(i) += B1.cwiseProduct(adjGdC.block<6, 2>(6 * i + 3, 0)).sum();
    }

    negVel = -(b.row(6 * N - 5) + 2.0 * T1(N - 1) * b.row(6 * N - 4) +
               3.0 * T2(N - 1) * b.row(6 * N - 3) +
               4.0 * T3(N - 1) * b.row(6 * N - 2) +
               5.0 * T4(N - 1) * b.row(6 * N - 1));
    negAcc = -(2.0 * b.row(6 * N - 4) + 6.0 * T1(N - 1) * b.row(6 * N - 3) +
               12.0 * T2(N - 1) * b.row(6 * N - 2) +
               20.0 * T3(N - 1) * b.row(6 * N - 1));
    negJer = -(6.0 * b.row(6 * N - 3) + 24.0 * T1(N - 1) * b.row(6 * N - 2) +
               60.0 * T2(N - 1) * b.row(6 * N - 1));

    B2 << negVel, negAcc, negJer;

    gdT(N - 1) += B2.cwiseProduct(adjGdC.block<3, 2>(6 * N - 3, 0)).sum();

    return;
  }

  template <typename EIGENMAT>
  inline void addPropCtoP(const Eigen::MatrixXd &adjGdC,
                          EIGENMAT &gdInP) const {
    for (int i = 0; i < N - 1; i++) {
      gdInP.col(i) += adjGdC.row(6 * i + 5).transpose();
    }
    return;
  }

  //@yuwei revised to vehicle case
  template <typename EIGENVEC>
  inline void addTimeIntPenalty(const Eigen::VectorXi cons,
                                const Eigen::VectorXi &idxHs,
                                const Eigen::Vector2d ci, double &cost,
                                EIGENVEC &gdT, Eigen::MatrixXd &gdC) const {
    double pena = 0.0;
    const double vmaxSqr = vmax_ * vmax_;
    const double amaxSqr = amax_ * amax_;
    const double kmaxSqr = kmax_ * kmax_;
    Eigen::Vector2d sigma, dsigma, ddsigma, dddsigma;
    double vel2, acc2, cur2;

    double step, alpha;
    double s1, s2, s3, s4, s5;
    Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3;
    Eigen::Vector2d outerNormal;
    int K;
    double violaPos, violaVel, violaAcc, violaCur;
    double violaPosPenaD, violaVelPenaD, violaAccPenaD, violaCurPenaD;
    double violaPosPena, violaVelPena, violaAccPena, violaCurPena;
    //@yuwei
    Eigen::Matrix<double, 6, 2> gradViolaVc, gradViolaAc, gradViolaKc;
    double gradViolaVt, gradViolaAt, gradViolaKt;
    double omg;

    Eigen::Matrix2d B_h;
    B_h << 0, -1, 1, 0;

    int innerLoop, idx;
    for (int i = 0; i < N; i++) {
      const auto &c = b.block<6, 2>(i * 6, 0);
      step = T1(i) / cons(i);  // resolution
      s1 = 0.0;
      innerLoop = cons(i) + 1;
      for (int j = 0; j < innerLoop; j++) {
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        beta0 << 1.0, s1, s2, s3, s4, s5;
        beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
        beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
        beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
        alpha = 1.0 / cons(i) * j;

        sigma = c.transpose() * beta0;
        dsigma = c.transpose() * beta1;
        ddsigma = c.transpose() * beta2;
        dddsigma = c.transpose() * beta3;

        // some help values
        double z_h0 = dsigma.transpose() * dsigma;
        double z_h1 = ddsigma.transpose() * dsigma;
        double z_h2 = dddsigma.transpose() * dsigma;
        double z_h3 = ddsigma.transpose() * B_h * dsigma;
        double z_h4 = z_h1 / z_h0;
        double z_h5 = z_h3 / (z_h0 * z_h0 * z_h0);

        vel2 = z_h0;
        acc2 = z_h1 * z_h1 / z_h0;
        cur2 = z_h3 * z_h3 / (z_h0 * z_h0 * z_h0);

        omg = (j == 0 || j == innerLoop - 1) ? 0.5 : 1.0;

        //@yuwei: add feasibility with curvature
        violaVel = vel2 - vmaxSqr;
        violaAcc = acc2 - amaxSqr;
        violaCur = cur2 - kmaxSqr;

        if (violaVel > 0.0) {
          violaVelPenaD = violaVel * violaVel;
          violaVelPena = violaVelPenaD * violaVel;
          violaVelPenaD *= 3.0;
          gradViolaVc = 2.0 * beta1 * dsigma.transpose();            // 6*2
          gradViolaVt = 2.0 * alpha * dsigma.transpose() * ddsigma;  // 1*1
          gdC.block<6, 2>(i * 6, 0) +=
              omg * step * ci(1) * violaVelPenaD * gradViolaVc;
          gdT(i) += omg * (ci(1) * violaVelPenaD * gradViolaVt * step +
                           ci(1) * violaVelPena / cons(i));
          pena += omg * step * ci(1) * violaVelPena;
        }

        if (violaAcc > 0.0) {
          violaAccPenaD = violaAcc * violaAcc;
          violaAccPena = violaAccPenaD * violaAcc;
          violaAccPenaD *= 3.0;
          //@yuwei
          gradViolaAc = 2.0 * beta2 *
                            (z_h4 * ddsigma.transpose() -
                             z_h4 * z_h4 * dsigma.transpose()) +
                        2.0 * beta3 * z_h4 * dsigma.transpose();  // 6*2
          gradViolaAt =
              2.0 * alpha *
              (z_h4 * (ddsigma.squaredNorm() + z_h2) - z_h4 * z_h4 * z_h1);
          gdC.block<6, 2>(i * 6, 0) +=
              omg * step * ci(2) * violaAccPenaD * gradViolaAc;
          gdT(i) += omg * (ci(2) * violaAccPenaD * gradViolaAt * step +
                           ci(2) * violaAccPena / cons(i));
          pena += omg * step * ci(2) * violaAccPena;
        }

        if (violaCur > 0.0) {
          violaCurPenaD = violaCur * violaCur;
          violaCurPena = violaCurPenaD * violaCur;
          violaCurPenaD *= 3.0;
          //@yuwei
          gradViolaKc =
              2.0 * beta2 *
                  (z_h5 * ddsigma.transpose() -
                   3 * z_h5 * z_h3 / z_h0 * dsigma.transpose()) +
              2.0 * beta3 * z_h5 * dsigma.transpose() * B_h.transpose();  // 6*2
          gradViolaKt = 2.0 * alpha *
                        (z_h5 * dddsigma.transpose() * B_h * dsigma -
                         3 * z_h5 * z_h3 / z_h0 * z_h1);
          gdC.block<6, 2>(i * 6, 0) +=
              omg * step * ci(3) * violaCurPenaD * gradViolaKc;
          gdT(i) += omg * (ci(3) * violaCurPenaD * gradViolaAt * step +
                           ci(3) * violaCurPena / cons(i));
          pena += omg * step * ci(3) * violaCurPena;
        }

        s1 += step;
      }
    }

    cost += pena;
    return;
  }

 public:
  inline void reset(const Eigen::MatrixXd &headState,
                    const Eigen::MatrixXd &tailState, const int &pieceNum) {
    N = pieceNum;
    headPVA = headState;
    tailPVA = tailState;
    T1.resize(N);

    A.create(6 * N, 6, 6);
    b.resize(6 * N, 2);
    gdC.resize(6 * N, 2);
    return;
  }

  inline void generate(const Eigen::MatrixXd &inPs, const Eigen::VectorXd &ts) {
    T1 = ts;
    T2 = T1.cwiseProduct(T1);
    T3 = T2.cwiseProduct(T1);
    T4 = T2.cwiseProduct(T2);
    T5 = T4.cwiseProduct(T1);

    A.reset();
    b.setZero();

    A(0, 0) = 1.0;
    A(1, 1) = 1.0;
    A(2, 2) = 2.0;
    b.row(0) = headPVA.col(0).transpose();
    b.row(1) = headPVA.col(1).transpose();
    b.row(2) = headPVA.col(2).transpose();

    for (int i = 0; i < N - 1; i++) {
      A(6 * i + 3, 6 * i + 3) = 6.0;
      A(6 * i + 3, 6 * i + 4) = 24.0 * T1(i);
      A(6 * i + 3, 6 * i + 5) = 60.0 * T2(i);
      A(6 * i + 3, 6 * i + 9) = -6.0;
      A(6 * i + 4, 6 * i + 4) = 24.0;
      A(6 * i + 4, 6 * i + 5) = 120.0 * T1(i);
      A(6 * i + 4, 6 * i + 10) = -24.0;
      A(6 * i + 5, 6 * i) = 1.0;
      A(6 * i + 5, 6 * i + 1) = T1(i);
      A(6 * i + 5, 6 * i + 2) = T2(i);
      A(6 * i + 5, 6 * i + 3) = T3(i);
      A(6 * i + 5, 6 * i + 4) = T4(i);
      A(6 * i + 5, 6 * i + 5) = T5(i);
      A(6 * i + 6, 6 * i) = 1.0;
      A(6 * i + 6, 6 * i + 1) = T1(i);
      A(6 * i + 6, 6 * i + 2) = T2(i);
      A(6 * i + 6, 6 * i + 3) = T3(i);
      A(6 * i + 6, 6 * i + 4) = T4(i);
      A(6 * i + 6, 6 * i + 5) = T5(i);
      A(6 * i + 6, 6 * i + 6) = -1.0;
      A(6 * i + 7, 6 * i + 1) = 1.0;
      A(6 * i + 7, 6 * i + 2) = 2 * T1(i);
      A(6 * i + 7, 6 * i + 3) = 3 * T2(i);
      A(6 * i + 7, 6 * i + 4) = 4 * T3(i);
      A(6 * i + 7, 6 * i + 5) = 5 * T4(i);
      A(6 * i + 7, 6 * i + 7) = -1.0;
      A(6 * i + 8, 6 * i + 2) = 2.0;
      A(6 * i + 8, 6 * i + 3) = 6 * T1(i);
      A(6 * i + 8, 6 * i + 4) = 12 * T2(i);
      A(6 * i + 8, 6 * i + 5) = 20 * T3(i);
      A(6 * i + 8, 6 * i + 8) = -2.0;

      b.row(6 * i + 5) = inPs.col(i).transpose();
    }

    A(6 * N - 3, 6 * N - 6) = 1.0;
    A(6 * N - 3, 6 * N - 5) = T1(N - 1);
    A(6 * N - 3, 6 * N - 4) = T2(N - 1);
    A(6 * N - 3, 6 * N - 3) = T3(N - 1);
    A(6 * N - 3, 6 * N - 2) = T4(N - 1);
    A(6 * N - 3, 6 * N - 1) = T5(N - 1);
    A(6 * N - 2, 6 * N - 5) = 1.0;
    A(6 * N - 2, 6 * N - 4) = 2 * T1(N - 1);
    A(6 * N - 2, 6 * N - 3) = 3 * T2(N - 1);
    A(6 * N - 2, 6 * N - 2) = 4 * T3(N - 1);
    A(6 * N - 2, 6 * N - 1) = 5 * T4(N - 1);
    A(6 * N - 1, 6 * N - 4) = 2;
    A(6 * N - 1, 6 * N - 3) = 6 * T1(N - 1);
    A(6 * N - 1, 6 * N - 2) = 12 * T2(N - 1);
    A(6 * N - 1, 6 * N - 1) = 20 * T3(N - 1);

    b.row(6 * N - 3) = tailPVA.col(0).transpose();
    b.row(6 * N - 2) = tailPVA.col(1).transpose();
    b.row(6 * N - 1) = tailPVA.col(2).transpose();

    A.factorizeLU();
    A.solve(b);

    return;
  }

  inline const Eigen::MatrixXd &get_b() const { return b; }

  inline const Eigen::VectorXd &get_T1() const { return T1; }

  inline Eigen::MatrixXd &get_gdC() { return gdC; }

  inline double getTrajJerkCost() const {
    double objective = 0.0;
    for (int i = 0; i < N; i++) {
      objective += 36.0 * b.row(6 * i + 3).squaredNorm() * T1(i) +
                   144.0 * b.row(6 * i + 4).dot(b.row(6 * i + 3)) * T2(i) +
                   192.0 * b.row(6 * i + 4).squaredNorm() * T3(i) +
                   240.0 * b.row(6 * i + 5).dot(b.row(6 * i + 3)) * T3(i) +
                   720.0 * b.row(6 * i + 5).dot(b.row(6 * i + 4)) * T4(i) +
                   720.0 * b.row(6 * i + 5).squaredNorm() * T5(i);
    }
    return objective;
  }

  // zx version add
  template <typename EIGENVEC, typename EIGENMAT>
  inline void getGrad2TP(EIGENVEC &gdT,
                         EIGENMAT &gdInPs)  // gradP
  {
    solveAdjGradC(gdC);
    addPropCtoT(gdC, gdT);
    addPropCtoP(gdC, gdInPs);
  }

  // initAndGetSmoothnessGradCost2PT
  template <typename EIGENVEC>
  inline void initGradCost(EIGENVEC &gdT, double &cost) {
    // printf( "gdInPs=%d\n", gdInPs.size() );
    gdT.setZero();
    gdC.setZero();
    cost = getTrajJerkCost();

    addGradJbyT(gdT);
    addGradJbyC(gdC);
  }

  template <typename EIGENVEC, typename EIGENMAT>
  inline void evalTrajCostGrad(const Eigen::VectorXi &cons,
                               const Eigen::VectorXi &idxHs, const double &vmax,
                               const double &amax, const double &kmax,
                               const Eigen::Vector2d &ci, double &cost,
                               EIGENVEC &gdT, EIGENMAT &gdInPs) {
    gdT.setZero();
    gdInPs.setZero();
    gdC.setZero();
    // smoo_cost
    cost = getTrajJerkCost();
    addGradJbyT(gdT);
    addGradJbyC(gdC);

    vmax_ = vmax;
    amax_ = amax;
    kmax_ = kmax;

    addTimeIntPenalty(cons, idxHs, ci, cost, gdT, gdC);  // no cfgHs

    solveAdjGradC(gdC);
    addPropCtoT(gdC, gdT);
    addPropCtoP(gdC, gdInPs);
  }

  inline Trajectory<5, 2> getTraj(int s) const {
    Trajectory<5, 2> traj;
    traj.reserve(N);
    for (int i = 0; i < N; i++) {
      traj.emplace_back(
          T1(i), b.block<6, 2>(6 * i, 0).transpose().rowwise().reverse(), s);
    }

    return traj;
  }

  inline Eigen::MatrixXd getInitConstraintPoints(const int K) const {
    Eigen::MatrixXd pts(2, N * K + 1);
    Eigen::Vector2d pos;
    Eigen::Matrix<double, 6, 1> beta0;
    double s1, s2, s3, s4, s5;
    double step;
    int i_dp = 0;

    for (int i = 0; i < N; ++i) {
      const auto &c = b.block<6, 2>(i * 6, 0);
      step = T1(i) / K;
      s1 = 0.0;
      // innerLoop = K;

      for (int j = 0; j <= K; ++j) {
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        beta0 << 1.0, s1, s2, s3, s4, s5;
        pos = c.transpose() * beta0;
        pts.col(i_dp) = pos;

        s1 += step;
        if (j != K || (j == K && i == N - 1)) {
          ++i_dp;
        }
      }
    }

    return pts;
  }
};
}  // namespace minco_local_planner::minco

#endif
