#include "Core/InEKF.h"

namespace InEKF {

template <class pM>
typename InEKF<pM>::Group InEKF<pM>::Predict(const U& u, double dt){    
    // Predict mu
    MatrixCov Sigma = state_.Cov();
    state_ = pModel.f(u, dt, state_);

    // Predict Sigma
    MatrixCov Phi = pModel.MakePhi(u, dt, state_, error_);

    MatrixCov Q = pModel.getQ();
    if(error_ == ERROR::RIGHT){
        MatrixCov Adj_X = Group::Ad( state_() );
        Q = Adj_X*Q*Adj_X.transpose();
    }
    Sigma = Phi* (Sigma + Q*dt) *Phi.transpose();
    state_.setCov( Sigma );

    return state_;
}


template <class pM>
typename InEKF<pM>::Group InEKF<pM>::Update(const Eigen::VectorXd& z, std::string type, MatrixH H){
    mModels[type].setH(H);
    Update(z, type);
}

template <class pM>
typename InEKF<pM>::Group InEKF<pM>::Update(const Eigen::VectorXd& z, std::string type){
    MeasureModel<Group> * m_model = mModels[type]; 

    // Change H via adjoint if necessary
    Eigen::MatrixXd H = m_model->getHBase();
    if( error_ != m_model->getError() ){
        if(error_ == ERROR::RIGHT){
            H *= Group::Ad( state_.getMu().inverse() );
        }
        else{
            H *= Group::Ad( state_.getMu() );
        }
    }
    m_model->setH( H );
    
    // Use measurement model to make Sinv and V
    m_model->Observe(z, state_);
    Eigen::VectorXd V = m_model->getV();
    Eigen::MatrixXd Sinv = m_model->getSinv();

    // Caculate K
    Eigen::MatrixXd K = state_.getSigma() * H.transpose() * Sinv;
    Eigen::VectorXd dState = K * V;

    // Apply to all states
    Eigen::MatrixXd dX = m_model->lie_->ExpMountain( dState.head(m_model->lie_->getMuStates()) );
    if(state_.error == ERROR::RIGHT){
        state_.setMu(dX  * state_.getMu());  
    }
    else{
        state_.setMu(state_.getMu() * dX);  
    }

    if(m_model->lie_->getAugmentSize() != 0){
        state_.setAugment( state_.getAugment() + dState.tail(m_model->lie_->getAugmentSize()) );  
    }

    int dimSigma = state_.getSigma().rows();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(dimSigma, dimSigma);
    state_.setSigma( (I - K*H)*state_.getSigma() );

    return state_;
}

template <class pM>
void InEKF<pM>::addMeasureModel(std::string name, MeasureModel<Group>* m){
    mModels[name] = m;
}

template <class pM>
void InEKF<pM>::addMeasureModels(std::map<std::string, MeasureModel<Group>*> m){
    mModels.insert(m.begin(), m.end());
}

}