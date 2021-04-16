#include "SE2_3_Bias/DepthSensor.h"

namespace InEKF {

DepthSensor::DepthSensor() {
    M_ = Eigen::Matrix3d::Zero();
    error_ = ERROR::LEFT;
    H_base_ = Eigen::Matrix<double, 3, 15>::Zero();
    H_base_.block<3,3>(0,6) = Eigen::Matrix3d::Identity();
    lie_ = new SE2_3_Bias;
}

void DepthSensor::setNoise(double std){
    // Actually storing M.inverse() here
    M_ = Eigen::Matrix3d::Zero();
    M_(2,2) = 1 / (std*std);
}

void DepthSensor::Observe(const Eigen::VectorXd& z, const State& state){
    // Find V
    Eigen::Vector<double, 5> z_full;
    Eigen::Vector3d p = state[2];
    z_full << p[0], p[1], z[0], 0, 1;
    V_ = (state.getMu().inverse() * z_full).head(3);

    // Calculate Sinv
    Eigen::Matrix3d R = state[0];
    Eigen::Matrix3d Sig = (H_*state.getSigma()*H_.transpose()).inverse();
    Sinv_ = Sig - Sig*( R.transpose()*M_*R + Sig ).inverse()*Sig;
}

}