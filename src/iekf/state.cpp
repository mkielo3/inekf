#include "iekf/state.h"

// Defaults to SE(3)
State::State() : State(3, 1) {}

State::State(LieGroup* lie) : State(lie->getDim(), lie->getCols(), lie->getAugment()) {}

State::State(int dim, int cols, int augment) : dim(dim), cols(cols), augment(augment) {
    Mu_ = Eigen::MatrixXd::Identity(dim+cols, dim+cols);
    Sigma_ = Eigen::MatrixXd::Zero(dim+cols*dim+augment, dim+cols*dim+augment);
    Augment_ = Eigen::VectorXd::Zero(augment);
}

Eigen::Ref<Eigen::MatrixXd> State::operator[](int idx){
    if(idx >= dim+cols) throw 2;
    if(idx < 0 ) throw 2;
    if(idx == 0){
        return Mu_.block<3,3>(0,0);
    }
    else{
        return Mu_.block<3,1>(0,2+idx);
    }
}

Eigen::MatrixXd State::operator[](int idx) const{
    if(idx >= dim+cols) throw 2;
    if(idx < 0 ) throw 2;
    if(idx == 0){
        return Mu_.block<3,3>(0,0);
    }
    else{
        return Mu_.block<3,1>(0,2+idx);
    }
}

const Eigen::Matrix3d State::getRotation(){
    return Mu_.block<3,3>(0,0);
}

const Eigen::MatrixXd State::getMu(){
    return Mu_;
}

const Eigen::MatrixXd State::getSigma(){
    return Sigma_;
}

const Eigen::VectorXd State::getAugment(){
    return Augment_;
}

const Eigen::VectorXd State::getLastu(){
    return Last_u_;
}


void State::setRotation(Eigen::Matrix3d R){
    Mu_.block<3,3>(0,0) = R;
}

void State::setLastu(Eigen::VectorXd u){
    Last_u_ = u;
}

void State::setMu(Eigen::MatrixXd Mu){
    Mu_ = Mu;
}

void State::setSigma(Eigen::MatrixXd Sigma){
    Sigma_ = Sigma;
}

void State::setAugment(Eigen::VectorXd Augment){
    Augment_ = Augment;
}