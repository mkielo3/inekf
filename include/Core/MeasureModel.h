#ifndef BASE_MEASURE
#define BASE_MEASURE

#include <Eigen/Core>
#include <Eigen/LU>
#include "Core/LieGroup.h"

namespace InEKF {

template<class Group>
class MeasureModel {
    
    public:
        typedef Eigen::Matrix<double,Group::rotSize,Group::rotSize> MatrixS;
        typedef Eigen::Matrix<double,Group::rotSize,Group::N> MatrixH;
        typedef Eigen::Matrix<double,Group::rotSize,1> VectorV;
        typedef Eigen::Matrix<double,Group::M,1> VectorB;

    protected:
        // These are all constant and should be set once
        ERROR error_;
        MatrixS M_ = MatrixS::Identity(Group::rotSize, Group::rotSize);

        // This one can be changed each iteration in InEKF.Update, 
        // or should be set once in constructor
        VectorB b_ = VectorB::Zero(Group::m, 1);
        MatrixH H_ = MatrixH::Zero(Group::rotSize, Group::c);

        // This is changed by InEKF based on if it's a RIGHT/LEFT filter
        // Use this in calcSInverse if you override it
        MatrixH H_error_;


    public: 
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
               
        MeasureModel() {};
        MeasureModel(VectorB b, const MatrixS& M, ERROR error) : b_(b) {
            if(Group::N == Eigen::Dynamic){
                throw std::range_error("Can't use Base MeasureModel on group with dynamic columns");
            }

            // Plug in constants
            this->M_ = M;
            this->error_ = error;

            this->H_ = MatrixH::Zero(Group::rotSize, Group::N);

            // Get rotation dimension
            int rDim = Group::rotSize*(Group::rotSize - 1) / 2;

            // Fill out rotation portion of H
            // SO(2) / SE(2)
            if(rDim == 1){
                this->H_(0,0) = -b_(1);
                this->H_(1,0) = b_(0);
            }
            // SO(3) / SE(3)
            else if(rDim == 3){
                this->H_.block(0, 0, Group::rotSize, Group::rotSize) = -1*SO3<>::Wedge(b_.head(3));
            }

            // Fill out column portion of H
            for(int i=0; i<Group::M-Group::rotSize; i++){
                this->H_.block(0, Group::rotSize*i+rDim, Group::rotSize, Group::rotSize) = b_(i+Group::rotSize)*MatrixS::Identity();
            }
            if(error == ERROR::RIGHT){
                this->H_ *= -1;
            }
        }

        virtual VectorB processZ(const Eigen::VectorXd& z, const Group& state) {
            if(z.rows() == Group::M){
                return z;
            }
            else if(z.rows() == Group::rotSize){
                VectorB temp = b_;
                temp.head(Group::rotSize) = z;
                return temp;
            }
            else{
                throw std::range_error("Wrong sized z");
            }
        }

        virtual MatrixH makeHError(const Group& state, ERROR iekfERROR){
            if( iekfERROR != error_ ){
                if(iekfERROR == ERROR::RIGHT){
                    H_error_ = H_*Group::Ad( state.inverse()() );
                }
                else{
                    H_error_ = H_*Group::Ad( state() );
                }
            }
            else{
                H_error_ = H_;
            }
            return H_error_;
        }

        virtual VectorV calcV(const VectorB& z, const Group& state){
            // calculate V
            VectorV V;
            if(error_ == ERROR::RIGHT){
                V.noalias() = state().block(0,0,Group::rotSize,state().cols()) * z - b_.head(Group::rotSize);
            }
            else{
                V.noalias() = state.inverse()().block(0,0,Group::rotSize,state().cols()) * z - b_.head(Group::rotSize);
            }
            return V;
        }

        virtual MatrixS calcSInverse(const Group& state){
            MatrixS Sinv;
            MatrixS R = state.R()();
            if(error_ == ERROR::RIGHT){
                Sinv.noalias() = ( H_error_*state.Cov()*H_error_.transpose() + R*M_*R.transpose() ).inverse();
            }
            else{
                Sinv.noalias() = ( H_error_*state.Cov()*H_error_.transpose() + R.transpose()*M_*R ).inverse();
            }
            return Sinv;
        }

        MatrixH getH() { return H_; }
        ERROR getError() { return error_; }

        void setH(MatrixH H) { H_ = H; }
};

}

#endif // BASE_MEASURE