#include "models/dvl.h"

DVLSensor::DVLSensor(){
    M_ = Eigen::MatrixXd::Zero(3,3);
    H_ = Eigen::MatrixXd::Zero(3,15);
    H_.block<3,3>(0,3) = Eigen::MatrixXd::Identity(3,3);
}

void DVLSensor::setNoise(double std){
    M_ = Eigen::MatrixXd::Identity(3, 3) * std*std;
}

void DVLSensor::Observe(Eigen::VectorXd& z, State& state){
    // Find V
    Eigen::VectorXd z_full(5);
    z_full << z[0], z[1], z[2], -1, 0;
    V_ = (state.getMu() * z_full).head(3);

    // Calculate Sinv
    Eigen::Matrix3d R = state.getRotation();
    Sinv_ = ( H_*state.getSigma()*H_.transpose() + R*M_*R.transpose() ).inverse();
}