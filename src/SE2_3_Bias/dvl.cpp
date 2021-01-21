#include "SE2_3_Bias/dvl.h"

namespace InEKF {

DVLSensor::DVLSensor(Eigen::Matrix3d dvl_r, Eigen::Vector3d dvl_p)
    : dvl_r(dvl_r) {
    M_ = Eigen::MatrixXd::Zero(3,3);
    error_ = State::RIGHT;
    H_base_ = Eigen::MatrixXd::Zero(3,15);
    H_base_.block<3,3>(0,3) = Eigen::MatrixXd::Identity(3,3);
    lie_ = new SE2_3_Bias;
    this->dvl_p = lie_->Cross(dvl_p);
}

DVLSensor::DVLSensor() 
    : DVLSensor(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()) {
}

void DVLSensor::setNoise(double std_dvl, double std_imu){
    M_ = Eigen::MatrixXd::Identity(3, 3) * std_dvl*std_dvl;
    
    // Rotate into IMU frame
    Eigen::Matrix3d IMU = Eigen::MatrixXd::Identity(3, 3) * std_imu*std_imu;
    M_ = dvl_r*M_*dvl_r.transpose() + dvl_p*IMU*dvl_p.transpose();
}

void DVLSensor::Observe(const Eigen::VectorXd& z, const State& state){
    // Convert to IMU frame
    Eigen::VectorXd z_full(5);
    z_full << z[0], z[1], z[2], -1, 0;

    Eigen::Vector3d omega = state.getLastu().head(3);
    z_full.head(3) = dvl_r*z_full.head(3) + dvl_p*omega;

    // Find V
    V_ = (state.getMu() * z_full).head(3);

    // Calculate Sinv
    Eigen::Matrix3d R = state[0];
    Sinv_ = ( H_*state.getSigma()*H_.transpose() + R*M_*R.transpose() ).inverse();
}

}