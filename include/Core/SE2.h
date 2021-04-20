#ifndef CLASS_SE2
#define CLASS_SE2

#include <Eigen/Core>
#include "LieGroup.h"
#include "SO2.h"
#include "iostream"

namespace InEKF {

template <int cols=1, int aug=0>
class SE2 : public LieGroup<SE2<cols,aug>,calcStateDim(2,cols,aug),calcStateMtxSize(2,cols)>{
    public:
        static constexpr int rotSize = 2;
        static constexpr int dimension = calcStateDim(rotSize,cols,aug);
        static constexpr int mtxSize = calcStateMtxSize(rotSize,cols);

        typedef typename LieGroup<SE2<cols,aug>,dimension,mtxSize>::TangentVector TangentVector;
        typedef typename LieGroup<SE2<cols,aug>,dimension,mtxSize>::MatrixCov MatrixCov;
        typedef typename LieGroup<SE2<cols,aug>,dimension,mtxSize>::MatrixState MatrixState;
        typedef Eigen::Matrix<double, aug, 1> VectorAug;

    private:
        // dummies to help with dynamic initialization
        static constexpr int a = aug == Eigen::Dynamic ? 0 : aug;
        static constexpr int c = dimension == Eigen::Dynamic ? 3 : dimension;
        static constexpr int m = mtxSize == Eigen::Dynamic ? 3 : mtxSize;

        MatrixState State_;
        MatrixCov Cov_;
        VectorAug Aug_;
        bool isUncertain;

        void verifySize();
        static void verifyTangentVector(const TangentVector& xi);

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        // Constructors
        SE2(const MatrixState& State=MatrixState::Identity(m,m), 
            const MatrixCov& Cov=MatrixCov::Zero(c,c),
            const VectorAug& Aug=VectorAug::Zero(a,1))
                : State_(State), Cov_(Cov), Aug_(Aug), isUncertain(!Cov.isZero()) { verifySize(); }
        SE2(bool uncertain) : SE2() {
            Cov_ = MatrixCov::Identity(c,c);
            isUncertain = uncertain;
        }
        SE2(const SE2& State) : SE2(State(), State.Cov(), State.Aug()) {}
        SE2(const TangentVector& xi,
            const MatrixCov& Cov=MatrixCov::Zero(c,c));
        SE2(double theta, double x, double y,
            const MatrixCov& Cov=MatrixCov::Zero(c,c),
            const VectorAug& Aug=VectorAug::Zero(a,1));
        ~SE2() {}

        // Getters
        bool Uncertain() const { return isUncertain; }
        MatrixCov Cov() const { return Cov_; }
        VectorAug Aug() const { return Aug_; }
        MatrixState operator()() const { return State_; }
        SO2<> R() const { 
            Eigen::Matrix2d R = State_.block(0,0,2,2);
            return SO2<>(R); 
        }
        Eigen::Vector2d operator[](int idx) const { return State_.block(0,2+idx,2,1); }
        
        // Setters
        void set(MatrixState S) { State_ = S; }
        void setCov(MatrixCov Cov) { Cov_ = Cov; }
        void setAug(MatrixCov Aug) { Aug_ = Aug; }

        void addCol(const Eigen::Vector2d& x, const Eigen::Matrix2d& sigma=Eigen::Matrix2d::Identity());
        void addAug(double x, double sigma=1);

        // Self operations
        SE2 inverse() const;
        using LieGroup<SE2<cols,aug>,dimension,mtxSize>::Ad;
        using LieGroup<SE2<cols,aug>,dimension,mtxSize>::log;

        // Group action
        SE2 operator*(const SE2& rhs) const;

        // Static Operators
        static MatrixState Wedge(const TangentVector& xi);
        static SE2 Exp(const TangentVector& xi);
        static TangentVector Log(const SE2& g);
        static MatrixCov Ad(const SE2& g);

};

}

#include "SE2.tpp"

#endif // CLASS_SE2