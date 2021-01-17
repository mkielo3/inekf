#include "iekf/iekf.h"

State InEKF::Update(Eigen::VectorXd u, double dt){
    // Update mu
    p_model_->f(u, dt, state_);

    // Update Sigma
    Eigen::MatrixXd Adj_X = p_model_->lie_->Adjoint( state_.getMu() );
    Eigen::MatrixXd Phi   = p_model_->MakePhi(u, dt, state_);

    Eigen::MatrixXd Sigma = Phi* (state_.getSigma() + Adj_X*p_model_->getQ()*Adj_X.transpose()*dt) *Phi.transpose();
    state_.setSigma( Sigma );

    return state_;
}

State InEKF::Correct(Eigen::VectorXd z, std::string type){
    return state_;
}

void InEKF::setProcessModel(ProcessModel& p){
    p_model_ = &p;
}

void InEKF::addMeasureModel(MeasureModel& m, std::string name){
    m_models_[name] = &m;
}