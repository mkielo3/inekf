#include "gtest/gtest.h"
#include <Eigen/Dense>
#include <InEKF/Core>
#include <InEKF/SE2_SLAM>

bool MatrixEquality(const Eigen::MatrixXd &lhs, const Eigen::MatrixXd &rhs) {
  return lhs.isApprox(rhs, 1e-6);
}

TEST(OdometryProcess, f){
    InEKF::SE2<> state;
    InEKF::SE2<> U(.1, 1, 2);

    InEKF::OdometryProcess op;

    EXPECT_PRED2(MatrixEquality, op.f(U, 1, state)(), U());
}

TEST(OdometryProcess, MakePhi){
    InEKF::SE2<> state;
    InEKF::SE2<> U(.1, 1, 2);

    InEKF::OdometryProcess op;
    Eigen::Matrix3d Phi;

    // check right
    Phi = op.MakePhi(U, 1, state, InEKF::RIGHT);
    EXPECT_PRED2(MatrixEquality, Phi, Eigen::Matrix3d::Identity());

    // check left
    Phi = op.MakePhi(U, 1, state, InEKF::LEFT);
    EXPECT_PRED2(MatrixEquality, Phi, U.Ad());
}