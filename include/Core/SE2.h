#ifndef CLASS_SE2
#define CLASS_SE2

#include <Eigen/Dense>
#include "LieGroup.h"
#include "SE2.h"

namespace InEKF {

template <int cols=1, int aug=0>
class SE2 : public LieGroup<SE2<cols,aug>, calcStateDim(2,cols,aug)>{
    private:
        typedef typename LieGroup<SE2<cols,aug>,calcStateDim(2,cols,aug)>::TangentVector TangentVector;
        typedef Eigen::Matrix<double, calcStateMtxSize(2,cols), calcStateMtxSize(2,cols)> MatrixState;
        typedef Eigen::Matrix<double, calcStateDim(2,cols,aug), calcStateDim(2,cols,aug)> MatrixCov;
        
        MatrixState State;
        MatrixCov Cov;
        Eigen::Matrix<double, aug, 1> Aug;


    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        SE2() {}
        // TODO: Figure out constructors with multiple cols
        // TODO: Function to add columns/augment state in
        SE2(const SE2& State_) : State(State_()), Cov(State_.getCov()), Aug(State_.getAug()) {}
        SE2(const MatrixState& State_, 
            const MatrixCov Cov_=MatrixCov::Identity(),
            const Eigen::Matrix<double,aug,1> Aug_ = Eigen::Matrix<double,aug,1>::Zeros()) 
                : State(State_), Cov(Cov_), Aug(Aug_) {}
        SE2(double theta, double x, double y,
            const MatrixCov Cov_=MatrixCov::Identity(),
            const Eigen::Matrix<double,aug,1> Aug_ = Eigen::Matrix<double,aug,1>::Zeros()) 
                : Cov(Cov_), Aug(Aug_)  {
            State << cos(theta), -sin(theta), x,
                     sin(theta),  cos(theta), y,
                     0, 0, 1;
        }
        SE2(const SO2<>& R, const Eigen::Vector2d& t,
            const MatrixCov Cov_=MatrixCov::Identity(),
            const Eigen::Matrix<double,aug,1> Aug_ = Eigen::Matrix<double,aug,1>::Zeros()) 
                : Cov(Cov_), Aug(Aug_)  {

            State = MatrixState::Identity();
            State.block<2,2>(0,0) = R();
            State.block<2,1>(0,2) = t;
        }
        ~SE2() {}

        MatrixState operator()() const
        {
            return State;
        }

        SE2 inverse() const{
            return SE2(State.inverse());
        }
        SE2 operator*(const SE2& rhs) const{
            return SE2( (*this)()*rhs() );
        }
        MatrixCov Ad(){
            return Ad(*this);
        }

        static SE2 Exp(const TangentVector& v){
            double theta = v(0);
            double x     = v(1);
            double y     = v(2);
            return SE2(theta, x, y);
        }
        static TangentVector Log(const SE2& g){
            TangentVector xi;
            xi << atan2(g()(1,0), g()(0,0));
            return xi;
        }
        static MatrixCov Ad(const SE2& g){
            return MatrixCov::Identity();
        }

};

}

#endif // CLASS_SE2