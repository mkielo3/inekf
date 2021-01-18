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
    MeasureModel * m_model = m_models_[type]; 
    m_model->Observe(z, state_);

    Eigen::VectorXd V = m_model->getV();
    Eigen::MatrixXd Sinv = m_model->getSinv();
    Eigen::MatrixXd H = m_model->getH();
    if( error_ != static_cast<ERROR>(m_model->getError()) ){
        if(error_ == InEKF::RIGHT){
            H *= m_model->lie_->Adjoint( state_.getMu().inverse() );
        }
        else{
            H *= m_model->lie_->Adjoint( state_.getMu() );
        }
    }

    Eigen::MatrixXd K = state_.getSigma() * H.transpose() * Sinv;
    Eigen::VectorXd dState = K * V;

    state_.setMu( p_model_->lie_->ExpMountain( dState.head(9) ) * state_.getMu() );  
    state_.setAugment( state_.getAugment() + dState.tail(6) );  

    int dimSigma = state_.getSigma().rows();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(dimSigma, dimSigma);
    state_.setSigma( (I - K*H)*state_.getSigma() );

    return state_;
}

void InEKF::setProcessModel(ProcessModel& p){
    p_model_ = &p;
}

void InEKF::addMeasureModel(MeasureModel& m, std::string name){
    m_models_[name] = &m;
}