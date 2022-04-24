#ifndef CLASS_SE2
#define CLASS_SE2

#include <Eigen/Core>
#include "LieGroup.h"
#include "SO2.h"

namespace InEKF {

/**
 * @brief 2D rigid body transformation, also known as the 3x3 special Euclidean group, SE(2).
 * 
 * @tparam C Number of Euclideans columns to include. Can be Eigen::Dynamic. Defaults to 1.
 * @tparam A Number of augmented Euclidean states. Can be Eigen::Dynamic if desired. Defaults to 0.
 */
template <int C=1, int A=0>
class SE2 : public LieGroup<SE2<C,A>,calcStateDim(2,C,A),calcStateMtxSize(2,C),A>{
    public:
        /**
        * @brief Size of rotational component of group
        * 
        */
        static constexpr int rotSize = 2;
        /**
         * @brief Dimension of group
         * 
         */
        static constexpr int N = calcStateDim(rotSize,C,A);
        /**
         * @brief State will have matrix of size M x M
         * 
         */
        static constexpr int M = calcStateMtxSize(rotSize,C);

        using typename LieGroup<SE2<C,A>,N,M,A>::TangentVector;
        using typename LieGroup<SE2<C,A>,N,M,A>::MatrixCov;
        using typename LieGroup<SE2<C,A>,N,M,A>::MatrixState;
        using typename LieGroup<SE2<C,A>,N,M,A>::VectorAug;

        // dummies to help with dynamic initialization
        /**
         * @brief Handles defaults values of augmented sizes when A is Eigen::Dyanmic.
         * 
         */
        static constexpr int a = A == Eigen::Dynamic ? 0 : A;
        /**
         * @brief Handles defaults values of tangent vector sizes when A is Eigen::Dyanmic.
         * 
         */
        static constexpr int c = N == Eigen::Dynamic ? 3 : N;
        /**
         * @brief Handles defaults values of matrix sizes.
         * 
         */
        static constexpr int m = M == Eigen::Dynamic ? 3 : M;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
        using LieGroup<SE2<C,A>,N,M,A>::Cov_;
        using LieGroup<SE2<C,A>,N,M,A>::State_;
        using LieGroup<SE2<C,A>,N,M,A>::Aug_;
        using LieGroup<SE2<C,A>,N,M,A>::isUncertain;

        void verifySize();
        static void verifyTangentVector(const TangentVector& xi);

    public:
        //------------ Constructors
        /**
         * @brief Construct SE2 object with all available options.
         * 
         * @param State An MxM Eigen matrix. Defaults to the identity.
         * @param Cov Covariance of state. If not input, state is set as "certain" and covariance is not tracked.
         * @param Aug Additional euclidean states if A != 0. Defaults to 0s. 
         */
        SE2(const MatrixState& State=MatrixState::Identity(m,m), 
            const MatrixCov& Cov=MatrixCov::Zero(c,c),
            const VectorAug& Aug=VectorAug::Zero(a,1))
                : LieGroup<SE2<C,A>,N,M,A>(State, Cov, Aug) { verifySize(); }
        
        // TODO: Turn this off for dynamic sizes
        // Used to make default uncertain state.. never used, hard to maintain.
        // SE2(bool uncertain) : SE2() {
        //     Cov_ = MatrixCov::Identity(c,c);
        //     isUncertain = uncertain;
        // }

        /**
         * @brief Copy constructor. Initialize with another SE2 object.
         * 
         * @param State SE2 object. The matrix, covariance and augmented state will all be copied from it.
         */
        SE2(const SE2& State) : SE2(State(), State.Cov(), State.Aug()) {}
        
        /**
         * @brief Construct a new SE2 object from a tangent vector using the exponential operator.
         * 
         * @param xi Tangent vector of size (1 + Augmented state size).
         * @param Cov Covariance of state. If not input, state is set as "certain" and covariance is not tracked.
         */
        SE2(const TangentVector& xi,
            const MatrixCov& Cov=MatrixCov::Zero(c,c));

        /**
         * @brief Construct a new SE2 object using an theta, x, y values. Only works if C=1.
         * 
         * @param theta Angle of rotate in radians.
         * @param x X-distance
         * @param y Y-distance
         * @param Cov Covariance of state. If not input, state is set as "certain" and covariance is not tracked.
         */
        SE2(double theta, double x, double y,
            const MatrixCov& Cov=MatrixCov::Zero(c,c)) { 
                throw std::invalid_argument("Can't use this constructor with those templates");
        }

        /**
         * @brief Destroy the SE2 object
         * 
         */
        ~SE2() {}

        //------------ Getters
        /**
         * @brief Gets rotational component of the state.
         * 
         * @return SO2<> Rotational component of the state.
         */
        SO2<> R() const { 
            Eigen::Matrix2d R = State_.block(0,0,2,2);
            return SO2<>(R); 
        }

        /**
         * @brief Gets the ith positional column of the group.
         * 
         * @param idx Index of column to get, from 0 to C-1.
         * @return Eigen::Vector2d 
         */
        Eigen::Vector2d operator[](int idx) const { 
            int curr_cols = State_.cols() - rotSize;
            if(idx >= curr_cols || idx < 0){
                throw std::out_of_range("Sliced out of range");
            }
            return State_.block(0,2+idx,2,1); 
        }

        /**
         * @brief Adds a column to the matrix state. Only usable if C = Eigen::Dynamic.
         * 
         * @param x Column to add in.
         * @param sigma Covariance of element. Only used if state is uncertain.
         */
        void addCol(const Eigen::Vector2d& x, const Eigen::Matrix2d& sigma=Eigen::Matrix2d::Identity());
        
        /**
         * @brief Adds an element to the augmented Euclidean state. Only usable if A = Eigen::Dynamic.
         * 
         * @param x Variable to add.
         * @param sigma Covariance of element. Only used if state is uncertain.
         */
        void addAug(double x, double sigma=1);

        //------------ Self operations
        /**
         * @brief Invert state.
         * 
         * @return Inverted matrix. Augmented portion and covariance is dropped.
         */
        SE2 inverse() const;
        using LieGroup<SE2<C,A>,N,M,A>::Ad;
        using LieGroup<SE2<C,A>,N,M,A>::log;

        //------------ Group action
        /**
         * @brief Combine transformations. Augmented states are summed.
         * 
         * @param rhs Right hand element of multiplication.
         * @return Combined elements with same augmented size.
         */
        SE2 operator*(const SE2& rhs) const;

        //------------ Static Operators
        /**
         * @brief Move element in R^n to the Lie algebra.
         * 
         * @param xi Tangent vector
         * @return MatrixState Element of Lie algebra
         */
        static MatrixState Wedge(const TangentVector& xi);

        /**
         * @brief Move an element from R^n -> algebra -> group
         * 
         * @param xi Tangent vector
         * @return Element of SE2
         */
        static SE2 Exp(const TangentVector& xi);

        /**
         * @brief Move an element from group -> algebra -> R^n
         * 
         * @param g Group element
         * @return TangentVector 
         */
        static TangentVector Log(const SE2& g);

        /**
         * @brief Compute the linear map Adjoint
         * 
         * @param g Element of SE2
         * @return Matrix of size state dimension x state dimension
         */
        static MatrixCov Ad(const SE2& g);

};

}

#include "SE2.tpp"

#endif // CLASS_SE2