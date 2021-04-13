#ifndef CLASS_SO2
#define CLASS_SO2

#include <Eigen/Dense>
#include "LieGroup.h"

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
        SO2() {}
        SO2(const MatrixState& State, 
            const MatrixCov Cov=MatrixCov::Zero(),
            const VectorAug Aug=VectorAug::Zero()) 
                : State_(State), Cov_(Cov), Aug_(Aug), isUncertain(Cov != MatrixCov::Zero()) {}
        SO2(const SO2& State) : SO2(State(), State.Cov(), State.Aug()) {}

        SO2(double theta, 
            const MatrixCov Cov=MatrixCov::Zero(),
            const VectorAug Aug=VectorAug::Zero()) {
            MatrixState State;
            State << cos(theta), -sin(theta),
                    sin(theta), cos(theta);
            SO2(State, Cov, Aug);
        }
        ~SO2() {}

        // Getters
        bool Uncertain() const { return isUncertain; }
        MatrixCov Cov() const { return Cov_; }
        VectorAug Aug() const { return Aug_; }
        MatrixState operator()() const { return State_; }

        // Self operations
        SO2<aug> inverse() const{
            return SO2(State_.transpose());
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
            if(isUncertain) Cov = this->Cov();
            if(rhs.Uncertain()) Cov = rhs.Cov();

            // Compose state + augment
            MatrixState State = (*this)() * rhs();
            VectorAug Aug = this->Aug() + rhs.Aug();

            return SO2<aug>(State, Cov, Aug);
        }

        // Static Operators
        static MatrixState Wedge(const TangentVector& xi){

        }
        static SO2<aug> Exp(const TangentVector& xi){
            double theta = xi(0);
            return SO2<aug>(theta,
                            MatrixCov::Zero(),
                            xi.segment(1,aug));
        }
        static TangentVector Log(const SO2<aug>& g){
            TangentVector xi;
            xi(0) = atan2(g()(1,0), g()(0,0));
            xi.segment(1, aug) = g.Aug();
            return xi;
        }
        static MatrixCov Ad(const SO2<aug>& g){
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