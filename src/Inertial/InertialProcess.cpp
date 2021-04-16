#include "SE2_3_Bias/InertialProcess.h"

namespace InEKF {

InertialProcess::InertialProcess()
    :  g_((Eigen::VectorXd(3) << 0,0,-9.81).finished()) {
    Q_ = MatrixCov::Zero();
}

static Group f(U u, double dt, Group state){
    // TODO: Clean this up
    // Get everything we need
    Eigen::Vector<double, 6> u_shifted = u - state.Aug();
    Eigen::Vector3d omega = u_shifted.head(3);
    Eigen::Vector3d a = u_shifted.tail(3);
    Eigen::Matrix3d R = state.R()();
    Eigen::Vector3d v = state[0];
    Eigen::Vector3d p = state[1];

    // Calculate
    // TODO: Make setters for a class
    state.R()() = R * lie_->ExpCross(omega*dt);
    state[0] = v + (R*a + g_)*dt;
    state[1] = p + v*dt + (R*a + g_)*dt*dt/2;

    // state.setLastu(u_shifted);
    return state;
}

static MatrixCov InertialProcess::MakePhi(const U& u, double dt, const Group& state, ERROR error){
    MatrixCov A = MatrixCov::Zero();

    // TODO: Figure out where error should be saved (probably in InEKF)
    if(error == ERROR::RIGHT){
        // Get everything we need
        Eigen::Matrix3d R = state.R()();
        Eigen::Matrix3d v_cross = SO3<>::Wedge( state[1] );
        Eigen::Matrix3d p_cross = SO3<>::Wedge( state[2] );

        A.block<3,3>(3,0) = SO3<>::Wedge(g_);
        A.block<3,3>(6,3) = Eigen::Matrix3d::Identity();
        
        A.block<3,3>(0,9) = -R;
        A.block<3,3>(3,9) = -v_cross * R;
        A.block<3,3>(6,9) = -p_cross * R;
        A.block<3,3>(3,12) = -R;
    }
    else{
        Eigen::Matrix3d w_cross = SO3<>::Wedge( u.head(3) );
        Eigen::Matrix3d a_cross = SO3<>::Wedge( u.tail(3) );

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
    Q_.block<3,3>(0,0) = Eigen::Matrix3d::Identity() * std*std;
}

void InertialProcess::setAccelNoise(double std){
    Q_.block<3,3>(3,3) = Eigen::Matrix3d::Identity() * std*std;
}

void InertialProcess::setGyroBiasNoise(double std){
    Q_.block<3,3>(9,9) = Eigen::Matrix3d::Identity() * std*std;
}

void InertialProcess::setAccelBiasNoise(double std){
    Q_.block<3,3>(12,12) = Eigen::Matrix3d::Identity() * std*std;
}

}