#include "iekf/iekf.h"

State InEKF::Predict(Eigen::VectorXd u, double dt){
    // Predict mu
    p_model_->f(u, dt, state_);

    // Predict Sigma
    Eigen::MatrixXd Adj_X = p_model_->lie_->Adjoint( state_.getMu() );
    Eigen::MatrixXd Phi   = p_model_->MakePhi(u, dt, state_);

    Eigen::MatrixXd Q = p_model_->getQ();
    if(state_.error == State::RIGHT){
        Q = Adj_X*Q*Adj_X.transpose();
    }
    Eigen::MatrixXd Sigma = Phi* (state_.getSigma() + p_model_->getQ()*dt) *Phi.transpose();
    state_.setSigma( Sigma );

    return state_;
}

State InEKF::Update(Eigen::VectorXd z, std::string type){
    MeasureModel * m_model = m_models_[type]; 
    m_model->Observe(z, state_);

    Eigen::VectorXd V = m_model->getV();
    Eigen::MatrixXd Sinv = m_model->getSinv();
    Eigen::MatrixXd H = m_model->getH();
    if( state_.error != m_model->getError() ){
        if(state_.error == State::RIGHT){
            H *= m_model->lie_->Adjoint( state_.getMu().inverse() );
        }
        else{
            H *= m_model->lie_->Adjoint( state_.getMu() );
        }
    }
    std::cout << V << std::endl;
    std::cout << Sinv << std::endl;
    std::cout << H << std::endl;

    Eigen::MatrixXd K = state_.getSigma() * H.transpose() * Sinv;
    Eigen::VectorXd dState = K * V;

    if(state_.error == State::RIGHT){
        state_.setMu( m_model->lie_->ExpMountain( dState.head(m_model->lie_->getMuStates()) ) * state_.getMu() );  
    }
    else{
        state_.setMu( state_.getMu() * m_model->lie_->ExpMountain( dState.head(m_model->lie_->getMuStates()) ) );  
    }

    if(m_model->lie_->getAugmentSize() != 0){
        state_.setAugment( state_.getAugment() + dState.tail(m_model->lie_->getAugmentSize()) );  
    }

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