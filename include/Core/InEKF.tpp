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
    MatrixH H = m_model->getH();
    if( error_ != m_model->getError() ){
        if(error_ == ERROR::RIGHT){
            H *= Group::Ad( state_.inverse()() );
        }
        else{
            H *= Group::Ad( state_() );
        }
    }
    m_model->setHError( H );

    // Use measurement model to make Sinv and V
    VectorV V = m_model->calcV(z, state_);
    MatrixS Sinv = m_model->calcSInverse(state_);

    // Caculate K + dX
    MatrixK K = state_.Cov() * H.transpose() * Sinv;
    TangentVector K_V = K * V;

    // Apply to states
    if(error_ == ERROR::RIGHT){
        state_ = Group::Exp(K_V) * state_;
    }
    else{
        state_ = state_ * Group::Exp(K_V);
    }

    MatrixCov I = MatrixCov::Identity();
    state_.setCov( (I - K*H)*state_.Cov() );

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