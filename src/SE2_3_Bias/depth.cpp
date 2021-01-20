#include "SE2_3_Bias/depth.h"

DepthSensor::DepthSensor() {
    M_ = Eigen::MatrixXd::Zero(3,3);
    error_ = State::LEFT;
    H_base_ = Eigen::MatrixXd::Zero(3,15);
    H_base_.block<3,3>(0,6) = Eigen::MatrixXd::Identity(3,3);
    lie_ = new SE2_3_Bias;
}

void DepthSensor::setNoise(double std){
    // Actually storing M.inverse() here
    M_ = Eigen::MatrixXd::Zero(3, 3);
    M_(2,2) = 1 / (std*std);
}

void DepthSensor::Observe(const Eigen::VectorXd& z, const State& state){
    // Find V
    Eigen::VectorXd z_full(5);
    Eigen::Vector3d p = state[2];
    z_full << p[0], p[1], z[0], 0, 1;
    V_ = (state.getMu().inverse() * z_full).head(3);

    // Calculate Sinv
    Eigen::Matrix3d R = state[0];
    Eigen::Matrix3d Sig = (H_*state.getSigma()*H_.transpose()).inverse();
    Sinv_ = Sig - Sig*( R.transpose()*M_*R + Sig ).inverse()*Sig;
}