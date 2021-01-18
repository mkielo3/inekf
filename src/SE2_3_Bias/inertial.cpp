#include "SE2_3_Bias/inertial.h"

InertialProcess::InertialProcess()
    :  g_((Eigen::VectorXd(3) << 0,0,-9.81).finished()) {
    Q_ = Eigen::MatrixXd::Zero(15,15);
    lie_ = new SE2_3_Bias;
}

void InertialProcess::f(Eigen::VectorXd u, double dt, State& state){
    // Get everything we need
    Eigen::Vector3d omega = u.head(3);
    Eigen::Vector3d a = u.tail(3);
    Eigen::MatrixXd R = state.getRotation();
    Eigen::Vector3d v = state.getVelocity();
    Eigen::Vector3d p = state.getPosition();

    // Calculate
    R = R * lie_->ExpCross(omega*dt);
    v = v + (R*a + g_)*dt;
    p = p + v*dt + (R*a + g_)*dt*dt/2;

    // Save it in our object
    state.setRotation(R);
    state.setVelocity(v);
    state.setPosition(p);

    state.setLastu(u);
}

Eigen::MatrixXd InertialProcess::MakePhi(Eigen::VectorXd u, double dt, State state){
    // Get everything we need
    Eigen::Matrix3d R = state.getRotation();
    Eigen::Matrix3d v_cross = lie_->Cross( state.getVelocity() );
    Eigen::Matrix3d p_cross = lie_->Cross( state.getPosition() );

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(15, 15);

    A.block<3,3>(3,0) = lie_->Cross(g_);
    A.block<3,3>(6,3) = Eigen::Matrix3d::Identity();
    
    A.block<3,3>(0,9) = -R;
    A.block<3,3>(3,9) = -v_cross * R;
    A.block<3,3>(6,9) = -p_cross * R;
    A.block<3,3>(3,12) = -R;

    return A.exp();
}

void InertialProcess::setGyroNoise(double std){
    Q_.block<3,3>(0,0) = Eigen::MatrixXd::Identity(3,3) * std*std;
}

void InertialProcess::setAccelNoise(double std){
    Q_.block<3,3>(3,3) = Eigen::MatrixXd::Identity(3,3) * std*std;
}

void InertialProcess::setGyroBiasNoise(double std){
    Q_.block<3,3>(9,9) = Eigen::MatrixXd::Identity(3,3) * std*std;
}

void InertialProcess::setAccelBiasNoise(double std){
    Q_.block<3,3>(12,12) = Eigen::MatrixXd::Identity(3,3) * std*std;
}