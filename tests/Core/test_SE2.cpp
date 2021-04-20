#include "gtest/gtest.h"
#include <Eigen/Dense>
#include <InEKF/Core>
#include "iostream"

TEST(SE2, BaseConstructor1){
    Eigen::Matrix4d state = Eigen::Matrix4d::Identity();
    Eigen::Matrix<double,7,7> sigma = Eigen::Matrix<double,7,7>::Identity();
    Eigen::Vector2d aug = Eigen::Vector2d::Ones();

    InEKF::SE2<2,2> x(state, sigma, aug);

    EXPECT_TRUE(x().isApprox(state));
    EXPECT_TRUE(x.Cov().isApprox(sigma));
    EXPECT_TRUE(x.Aug().isApprox(aug));
    EXPECT_TRUE(x.Uncertain());
}

TEST(SE2, BaseConstructor2){
    Eigen::MatrixXd state = Eigen::Matrix4d::Identity();
    Eigen::Matrix<double,6,6> sigma = Eigen::Matrix<double,6,6>::Identity();
    Eigen::Vector<double,1> aug = Eigen::Vector<double,1>::Ones();

    // make sure it gets mad if we pass the wrong sigma
    EXPECT_THROW(InEKF::SE2<Eigen::Dynamic> x(state, sigma);, std::range_error);
    // Or doesn't if we pass the right one
    EXPECT_NO_THROW(  (InEKF::SE2<Eigen::Dynamic,1>(state, sigma, aug)) );
    // dynamic aug also works
    Eigen::Matrix4d state2 = Eigen::Matrix4d::Identity();
    EXPECT_NO_THROW(  (InEKF::SE2<2,Eigen::Dynamic>(state2, sigma, aug)) );
}

TEST(SE2, TangentConstructor1){
    Eigen::Vector<double,6> x;
    x << 0, 1, 2, 3, 4, 5;

    InEKF::SE2<2,1> state(x);
    EXPECT_TRUE(state.R()().isApprox(Eigen::Matrix2d::Identity()));
    EXPECT_EQ(state()(0,2), 1);
    EXPECT_EQ(state()(1,2), 2);
    EXPECT_EQ(state()(0,3), 3);
    EXPECT_EQ(state()(1,3), 4);
    EXPECT_EQ(state.Aug()(0), 5);
}

TEST(SE2, TangentConstructor2){
    Eigen::VectorXd x(6);
    x << 0, 1, 2, 3, 4, 5;

    InEKF::SE2<Eigen::Dynamic,1> state(x);
    EXPECT_TRUE(state.R()().isApprox(Eigen::Matrix2d::Identity()));
    EXPECT_EQ(state()(0,2), 1);
    EXPECT_EQ(state()(1,2), 2);
    EXPECT_EQ(state()(0,3), 3);
    EXPECT_EQ(state()(1,3), 4);
    EXPECT_EQ(state.Aug()(0), 5);

    InEKF::SE2<2,Eigen::Dynamic> state2(x);
    EXPECT_TRUE(state2.R()().isApprox(Eigen::Matrix2d::Identity()));
    EXPECT_EQ(state2()(0,2), 1);
    EXPECT_EQ(state2()(1,2), 2);
    EXPECT_EQ(state2()(0,3), 3);
    EXPECT_EQ(state2()(1,3), 4);
    EXPECT_EQ(state2.Aug()(0), 5);

    // InEKF::SE2<Eigen::Dynamic,Eigen::Dynamic> state3(x);
    EXPECT_THROW( (InEKF::SE2<Eigen::Dynamic,Eigen::Dynamic>(x)), std::range_error);
}

TEST(SE2, PlainConstructor){
    InEKF::SE2<> x(0,1,2);
    EXPECT_TRUE(x.R()().isApprox(Eigen::Matrix2d::Identity()));
    EXPECT_EQ(x()(0,2), 1);
    EXPECT_EQ(x()(1,2), 2);
}

TEST(SE2, AddCol){
    InEKF::SE2<Eigen::Dynamic> x;
    EXPECT_TRUE(x().isApprox(Eigen::Matrix3d::Identity()));

    x.addCol(Eigen::Vector2d::Ones(2));

    EXPECT_EQ(x()(0,3), 1);
    EXPECT_EQ(x()(1,3), 1);

    // TODO: Test adding to Cov

    InEKF::SE2<> y;
    EXPECT_THROW( y.addCol(Eigen::Vector2d::Ones()), std::range_error);
}

TEST(SE2, AddAug){
    InEKF::SE2<1,Eigen::Dynamic> x;
    EXPECT_EQ(x.Aug().rows(), 0);

    x.addAug(2);

    EXPECT_EQ(x.Aug()(0), 2);

    // TODO: Test adding to Cov

    InEKF::SE2<> y;
    EXPECT_THROW( y.addAug(2), std::range_error);
}

// TESTS still to write: inverse, log, exp, wedge, aug