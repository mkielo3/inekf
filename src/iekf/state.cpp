#include "iekf/state.h"

const Eigen::Matrix3d State::getRotation(){
    return Mu_.block<3,3>(0,0);
}

const Eigen::Vector3d State::getVelocity(){
    return Mu_.block<3,1>(0,3);
}

const Eigen::Vector3d State::getPosition(){
    return Mu_.block<3,1>(0,4);
}

const Eigen::MatrixXd State::getMu(){
    return Mu_;
}

const Eigen::MatrixXd State::getSigma(){
    return Sigma_;
}

void State::setRotation(Eigen::Matrix3d R){
    Mu_.block<3,3>(0,0) = R;
}

void State::setVelocity(Eigen::Vector3d v){
    Mu_.block<3,1>(0,3) = v;
}

void State::setPosition(Eigen::Vector3d p){
    Mu_.block<3,1>(0,4) = p;
}
