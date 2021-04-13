#ifndef CLASS_SO2
#define CLASS_SO2

#include <Eigen/Dense>
#include "LieGroup.h"

namespace InEKF {

template<int aug=0>
class SO2 : public LieGroup<SO2<aug>,calcStateDim(2,0,aug)>{
    private:
        typedef typename LieGroup<SO2<aug>,calcStateDim(2,0,aug)>::TangentVector TangentVector;
        typedef Eigen::Matrix<double, calcStateMtxSize(2,0), calcStateMtxSize(2,0)> MatrixState;
        typedef Eigen::Matrix<double, calcStateDim(2,0,aug), calcStateDim(2,0,aug)> MatrixCov;
        
        MatrixState State;
        MatrixCov Cov;
        Eigen::Matrix<double, aug, 1> Aug;


    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        SO2() {}
        SO2(const SO2& State_) : State(State_()), Cov(State_.getCov()), Aug(State_.getAug()) {}
        SO2(const MatrixState& State_, 
            const MatrixCov Cov_=MatrixCov::Identity(),
            const Eigen::Matrix<double,aug,1> Aug_ = Eigen::Matrix<double,aug,1>::Zeros()) 
                : State(State_), Cov(Cov_), Aug(Aug_) {}
        SO2(double theta, 
            const MatrixCov Cov_=MatrixCov::Identity(),
            const Eigen::Matrix<double,aug,1> Aug_ = Eigen::Matrix<double,aug,1>::Zeros()) 
                : Cov(Cov_), Aug(Aug_)  {
            State << cos(theta), -sin(theta),
                    sin(theta), cos(theta);
        }
        ~SO2() {}

        MatrixCov getCov(){ return Cov; }
        MatrixCov getAug(){ return Aug; }

        MatrixState operator()() const
        {
            return State;
        }

        SO2 inverse() const{
            return SO2(State.transpose());
        }
        SO2 operator*(const SO2& rhs) const{
            return SO2( (*this)()*rhs() );
        }
        MatrixCov Ad(){
            return Ad(*this);
        }

        static SO2 Exp(const TangentVector& v){
            double theta = v(0);
            return SO2(theta);
        }
        static TangentVector Log(const SO2& g){
            TangentVector xi;
            xi << atan2(g()(1,0), g()(0,0));
            return xi;
        }
        static MatrixCov Ad(const SO2& g){
            return MatrixCov::Identity();
        }

};

template <int aug>
std::ostream& operator<<(std::ostream& os, const SO2<aug>& rhs)  
{
  os << rhs();
  return os;
}


}

#endif // CLASS_SO2