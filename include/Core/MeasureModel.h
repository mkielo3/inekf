#ifndef BASE_MEASURE
#define BASE_MEASURE

#include <Eigen/Core>
#include <Eigen/LU>
#include "Core/LieGroup.h"

namespace InEKF {

template<class Group>
class MeasureModel {
    
    protected:
        typedef Eigen::Matrix<double,Group::rotSize,Group::rotSize> MatrixS;
        typedef Eigen::Matrix<double,Group::rotSize,Group::N> MatrixH;
        typedef Eigen::Matrix<double,Group::rotSize,1> VectorV;
        typedef Eigen::Matrix<double,Group::M,1> VectorB;

        // These are all constant and should be set once
        ERROR error_;
        MatrixS M_;

        // This one can be changed each iteration in InEKF.Update, 
        // or should be set once in constructor
        MatrixH H_;

        // This is changed by InEKF based on if it's a RIGHT/LEFT filter
        // Use this in calcSInverse if you override it
        MatrixH H_error_;


    public: 
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
               
        MeasureModel() {};

        virtual VectorB processZ(const Eigen::VectorXd& z, const Group& state) { 
            if(z.rows() == Group::M){
                return z;
            }
            else{
                throw std::range_error("Wrong sized z");
            }
        }

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