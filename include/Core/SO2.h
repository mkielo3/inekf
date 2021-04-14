#ifndef CLASS_SO2
#define CLASS_SO2

#include <Eigen/Dense>
#include "LieGroup.h"
#include "iostream"

namespace InEKF {

template<int aug=0>
class SO2 : public LieGroup<SO2<aug>,calcStateDim(2,0,aug),2>{
    private:
        typedef typename LieGroup<SO2<aug>,calcStateDim(2,0,aug),2>::TangentVector TangentVector;
        typedef typename LieGroup<SO2<aug>,calcStateDim(2,0,aug),2>::MatrixCov MatrixCov;
        typedef typename LieGroup<SO2<aug>,calcStateDim(2,0,aug),2>::MatrixState MatrixState;
        typedef Eigen::Matrix<double, aug, 1> VectorAug;

        MatrixState State_;
        MatrixCov Cov_;
        VectorAug Aug_;
        bool isUncertain;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        // Constructors
        SO2(const MatrixState& State=MatrixState::Identity(), 
            const MatrixCov& Cov=MatrixCov::Zero(),
            const VectorAug& Aug=VectorAug::Zero()) 
                : State_(State), Cov_(Cov), Aug_(Aug), isUncertain(Cov != MatrixCov::Zero()) {}

        SO2(const SO2& State) : SO2(State(), State.Cov(), State.Aug()) {}

        SO2(double theta, 
            const MatrixCov& Cov=MatrixCov::Zero(),
            const VectorAug& Aug=VectorAug::Zero()) 
                : Cov_(Cov), Aug_(Aug), isUncertain(Cov != MatrixCov::Zero()){
            State_ << cos(theta), -sin(theta),
                    sin(theta), cos(theta);
        }

        SO2(const TangentVector& xi, const MatrixCov& Cov=MatrixCov::Zero())
            : SO2(xi(0), Cov, xi.segment(1,aug)) {}

        ~SO2() {}

        // Getters
        bool Uncertain() const { return isUncertain; }
        MatrixCov Cov() const { return Cov_; }
        VectorAug Aug() const { return Aug_; }
        MatrixState operator()() const { return State_; }

        // Self operations
        SO2<aug> inverse() const{
            MatrixState temp = State_.transpose();
            return SO2(temp);
        }
        using LieGroup<SO2<aug>,calcStateDim(2,0,aug),2>::Ad;
        using LieGroup<SO2<aug>,calcStateDim(2,0,aug),2>::log;

        // Group action
        SO2<aug> operator*(const SO2<aug>& rhs) const{
            // Skirt around composing covariances
            MatrixCov Cov = MatrixCov::Zero();
            if(this->Uncertain() && rhs.Uncertain()){
                throw "Can't compose uncertain LieGroups";
            }
            if(this->Uncertain()) Cov = this->Cov();
            if(rhs.Uncertain()) Cov = rhs.Cov();

            // Compose state + augment
            MatrixState State = (*this)() * rhs();
            VectorAug Aug = this->Aug() + rhs.Aug();

            return SO2<aug>(State, Cov, Aug);
        }

        // Static Operators
        static MatrixState Wedge(const TangentVector& xi){
            MatrixState State;
            double theta = xi(0);
            State << 0, -theta,
                    theta, 0;
            return State;
        }
        static SO2 Exp(const TangentVector& xi){
            double theta = xi(0);
            return SO2(theta,
                        MatrixCov::Zero(),
                        xi.segment(1,aug));
        }
        static TangentVector Log(const SO2& g){
            TangentVector xi;
            xi(0) = atan2(g()(1,0), g()(0,0));
            xi.segment(1, aug) = g.Aug();
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