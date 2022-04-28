#include "gtest/gtest.h"
#include <Eigen/Core>
#include <InEKF/Core>

#define EXPECT_MATRICES_EQ(M_actual, M_expected) \
  EXPECT_TRUE(M_actual.isApprox(M_expected, 1e-6)) << "  Actual:\n" << M_actual << "\nExpected:\n" << M_expected

// TODO: Test with dynamic types?
using Group = InEKF::SE2<2,1>;
using MatrixH = InEKF::MeasureModel<Group>::MatrixH;
using MatrixS = InEKF::MeasureModel<Group>::MatrixS;
using VectorV = InEKF::MeasureModel<Group>::VectorV;
using VectorB = InEKF::MeasureModel<Group>::VectorB;

TEST(MeasureModel, bConstructor){
    // make b, H and M
    VectorB b;
    b << 0, 0, 0, 1;
    MatrixH H = MatrixH::Zero();
    H.block(0,3,2,2) = Eigen::Matrix2d::Identity();
    MatrixS M = MatrixS::Identity();

    // Try with left error
    InEKF::MeasureModel<Group> l(b, M, InEKF::LEFT);
    EXPECT_MATRICES_EQ(l.getH(), H);

    // Try with right error
    InEKF::MeasureModel<Group> r(b, M, InEKF::RIGHT);
    EXPECT_MATRICES_EQ(r.getH(), (-1*H));

    // Try with things in top of b
    b(0) = 1;
    b(1) = 2;
    H(0,0) = -2;
    H(1,0) = 1;
    InEKF::MeasureModel<Group> with_b(b, M, InEKF::LEFT);
    EXPECT_MATRICES_EQ(with_b.getH(), H);

    // Try with SO3
    Eigen::Vector3d b2{1,2,3};
    Eigen::Matrix3d H2 = -1*InEKF::SO3<>::wedge(b2);
    InEKF::MeasureModel<InEKF::SO3<>> with_SO3(b2, Eigen::Matrix3d::Identity(), InEKF::LEFT);
    EXPECT_MATRICES_EQ(with_SO3.getH(), H2);
}

TEST(MeasureModel, processZ){
    // make b, H and M
    VectorB b;
    b << 0, 0, 0, 1;
    MatrixS M = MatrixS::Identity();
    Group state;
    InEKF::MeasureModel<Group> S(b, M, InEKF::LEFT);

    VectorV z;
    z << 2,2;

    VectorB expected;
    expected << 2,2,0,1;

    EXPECT_MATRICES_EQ(S.processZ(z, state), expected);
}