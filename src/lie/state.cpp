#include "lie/state.h"

const Eigen::Matrix3d State::getRotation(){
    return Mu_.block<3,3>(0,0);
}

const Eigen::Vector3d State::getVelocity(){
    return Mu_.block<3,1>(0,3);
}

const Eigen::Vector3d State::getPosition(){
    return Mu_.block<3,1>(0,3);
}
