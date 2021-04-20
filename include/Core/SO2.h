#ifndef CLASS_SO2
#define CLASS_SO2

#include <Eigen/Core>
#include "LieGroup.h"

namespace InEKF {

template<int aug=0>
class SO2 : public LieGroup<SO2<aug>,calcStateDim(2,0,aug),2,aug>{
    public:
        static constexpr int rotSize = 2;
        static constexpr int dimension = calcStateDim(rotSize,0,aug);
        static constexpr int mtxSize = calcStateMtxSize(rotSize,0);

        using typename LieGroup<SO2<aug>,dimension,mtxSize,aug>::TangentVector;
        using typename LieGroup<SO2<aug>,dimension,mtxSize,aug>::MatrixCov;
        using typename LieGroup<SO2<aug>,dimension,mtxSize,aug>::MatrixState;
        using typename LieGroup<SO2<aug>,dimension,mtxSize,aug>::VectorAug;

    private:
        // dummies to help with dynamic initialization
        static constexpr int a = aug == Eigen::Dynamic ? 0 : aug;
        static constexpr int c = aug == Eigen::Dynamic ? 1 : dimension;

        using LieGroup<SO2<aug>,dimension,mtxSize,aug>::Cov_;
        using LieGroup<SO2<aug>,dimension,mtxSize,aug>::State_;
        using LieGroup<SO2<aug>,dimension,mtxSize,aug>::Aug_;
        using LieGroup<SO2<aug>,dimension,mtxSize,aug>::isUncertain;
        
        void verifySize();

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        // Constructors
        SO2(const MatrixState& State=MatrixState::Identity(), 
            const MatrixCov& Cov=MatrixCov::Zero(c,c),
            const VectorAug& Aug=VectorAug::Zero(a)) 
                : LieGroup<SO2<aug>,dimension,mtxSize,aug>(State, Cov, Aug) { verifySize(); }

        SO2(const SO2& State) : SO2(State(), State.Cov(), State.Aug()) {}

        SO2(double theta, 
            const MatrixCov& Cov=MatrixCov::Zero(c,c),
            const VectorAug& Aug=VectorAug::Zero(a)) 
                : LieGroup<SO2<aug>,dimension,mtxSize,aug>(Cov, Aug) {
            State_ << cos(theta), -sin(theta),
                    sin(theta), cos(theta);
            verifySize();
        }

        SO2(const TangentVector& xi, const MatrixCov& Cov=MatrixCov::Zero(c,c))
            : SO2(xi(0), Cov, xi.tail(xi.size()-1)) {}

        ~SO2() {}

        // Getters
        SO2<> R() const { return SO2<>(State_); }

        // Self operations
        SO2<aug> inverse() const;
        using LieGroup<SO2<aug>,dimension,mtxSize,aug>::Ad;
        using LieGroup<SO2<aug>,dimension,mtxSize,aug>::log;

        // Group action
        SO2<aug> operator*(const SO2<aug>& rhs) const;

        // Static Operators
        static MatrixState Wedge(const TangentVector& xi);
        static SO2 Exp(const TangentVector& xi);
        static TangentVector Log(const SO2& g);
        static MatrixCov Ad(const SO2& g){ return MatrixCov::Identity(); }

};

template <int aug>
std::ostream& operator<<(std::ostream& os, const SO2<aug>& rhs)  
{
    os << "Matrix\n" << rhs();
    if(rhs.Uncertain()) os << "\nSigma\n" << rhs.Cov();
    if(aug != 0) os << "\nAug\n" << rhs.Aug();
    return os;
}


}

#include "SO2.tpp"

#endif // CLASS_SO2