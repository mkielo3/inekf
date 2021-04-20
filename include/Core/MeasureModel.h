#ifndef BASE_MEASURE
#define BASE_MEASURE

#include <Eigen/Dense>
#include "Core/LieGroup.h"

namespace InEKF {

template<class Group>
class MeasureModel {
    
    protected:
        typedef typename Eigen::Matrix<double,Group::rotSize,Group::rotSize> MatrixS;
        typedef Eigen::Matrix<double,Group::rotSize,Group::dimension> MatrixH;
        typedef Eigen::Matrix<double,Group::rotSize,1> VectorV;
        typedef Eigen::Matrix<double,Group::mtxSize,1> VectorB;

        // These are all constant and should be set once in the constructor
        ERROR error_;
        MatrixS M_;

        // This one can be changed each iteration, or should be set once in constructor
        MatrixH H_;

        // This is changed by InEKF based on if it's a RIGHT/LEFT filter
        MatrixH H_error_;


    public: 
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
               
        MeasureModel() {};

        virtual VectorB fillZ(const Eigen::VectorXd&, const Group& state) { throw 1; return VectorB::Zero(); }

        virtual VectorV calcV(const VectorB& z, const Group& state){
            // calculate V
            if(error_ == ERROR::RIGHT){
                return ( state() * z ).head(Group::rotSize);
            }
            else{
                return ( state.inverse()() * z ).head(Group::rotSize);
            }
        }

        virtual MatrixS calcSInverse(const Group& state){
            MatrixS Sinv;
            MatrixS R = state.R()();
            if(error_ == ERROR::RIGHT){
                Sinv = ( H_error_*state.Cov()*H_error_.transpose() + R*M_*R.transpose() ).inverse();
            }
            else{
                Sinv = ( H_error_*state.Cov()*H_error_.transpose() + R.transpose()*M_*R ).inverse();
            }
            return Sinv;
        }

        MatrixH getH() { return H_; }
        ERROR getError() { return error_; }

        void setH(MatrixH H) { H_ = H; }
        void setHError(MatrixH H) { H_error_ = H; }
};

}

#endif // BASE_MEASURE