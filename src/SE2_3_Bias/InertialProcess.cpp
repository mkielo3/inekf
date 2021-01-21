#include "SE2_3_Bias/InertialProcess.h"

namespace InEKF {

InertialProcess::InertialProcess()
    :  g_((Eigen::VectorXd(3) << 0,0,-9.81).finished()) {
    Q_ = Eigen::MatrixXd::Zero(15,15);
    lie_ = new SE2_3_Bias;
}

void InertialProcess::f(const Eigen::VectorXd& u, double dt, State& state){
    // Get everything we need
    Eigen::Vector3d omega = u.head(3);
    Eigen::Vector3d a = u.tail(3);
    Eigen::MatrixXd R = state[0];
    Eigen::Vector3d v = state[1];
    Eigen::Vector3d p = state[2];

    // Calculate
    state[0] = R * lie_->ExpCross(omega*dt);
    state[1] = v + (R*a + g_)*dt;
    state[2] = p + v*dt + (R*a + g_)*dt*dt/2;

    state.setLastu(u);
}

Eigen::MatrixXd InertialProcess::MakePhi(const Eigen::VectorXd& u, double dt, const State& state){
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(15, 15);

    if(state.error == ERROR::RIGHT){
        // Get everything we need
        Eigen::Matrix3d R = state[0];
        Eigen::Matrix3d v_cross = lie_->Cross( state[1] );
        Eigen::Matrix3d p_cross = lie_->Cross( state[2] );

        A.block<3,3>(3,0) = lie_->Cross(g_);
        A.block<3,3>(6,3) = Eigen::Matrix3d::Identity();
        
        A.block<3,3>(0,9) = -R;
        A.block<3,3>(3,9) = -v_cross * R;
        A.block<3,3>(6,9) = -p_cross * R;
        A.block<3,3>(3,12) = -R;
    }
    else{
        Eigen::Matrix3d w_cross = lie_->Cross( u.head(3) );
        Eigen::Matrix3d a_cross = lie_->Cross( u.tail(3) );

        A.block<3,3>(0,0) = -w_cross;
        A.block<3,3>(3,3) = -w_cross;
        A.block<3,3>(6,6) = -w_cross;
        A.block<3,3>(3,0) = -a_cross;
        A.block<3,3>(6,3) = Eigen::Matrix3d::Identity();

        A.block<3,3>(0,9) = -Eigen::Matrix3d::Identity();
        A.block<3,3>(3,12) = -Eigen::Matrix3d::Identity();
    }

    return (A*dt).exp();
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

}