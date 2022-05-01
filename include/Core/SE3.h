#ifndef CLASS_SE3
#define CLASS_SE3

#include <Eigen/Core>
#include "LieGroup.h"
#include "SO3.h"

namespace InEKF {

/**
 * @brief 3D rigid body transformation, also known as the 4x4 special Euclidean group, SE(3).
 * 
 * @tparam C Number of Euclideans columns to include. Can be Eigen::Dynamic. Defaults to 1.
 * @tparam A Number of augmented Euclidean states. Can be Eigen::Dynamic if desired. Defaults to 0.
 */
template <int C=1, int A=0>
class SE3 : public LieGroup<SE3<C,A>,calcStateDim(3,C,A),calcStateMtxSize(3,C),A>{
    public:
        /**
        * @brief Size of rotational component of group
        * 
        */
        static constexpr int rotSize = 3;
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

        using typename LieGroup<SE3<C,A>,N,M,A>::TangentVector;
        using typename LieGroup<SE3<C,A>,N,M,A>::MatrixCov;
        using typename LieGroup<SE3<C,A>,N,M,A>::MatrixState;
        using typename LieGroup<SE3<C,A>,N,M,A>::VectorAug;

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
        static constexpr int c = N == Eigen::Dynamic ? 6 : N;
        /**
         * @brief Handles defaults values of matrix sizes.
         * 
         */
        static constexpr int m = M == Eigen::Dynamic ? 4 : M;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 
    private:
        static constexpr int small_xi = N == Eigen::Dynamic ? Eigen::Dynamic : N-3;

        using LieGroup<SE3<C,A>,N,M,A>::Cov_;
        using LieGroup<SE3<C,A>,N,M,A>::State_;
        using LieGroup<SE3<C,A>,N,M,A>::Aug_;
        using LieGroup<SE3<C,A>,N,M,A>::isUncertain;

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
        SE3(const MatrixState& State=MatrixState::Identity(m,m), 
            const MatrixCov& Cov=MatrixCov::Zero(c,c),
            const VectorAug& Aug=VectorAug::Zero(a,1))
                : LieGroup<SE3<C,A>,N,M,A>(State, Cov, Aug) { verifySize(); }
        
        // // TODO: Turn this off for dynamic sizes
        // SE3(bool uncertain) : SE3() {
        //     Cov_ = MatrixCov::Identity(c,c);
        //     isUncertain = uncertain;
        // }

        /**
         * @brief Copy constructor. Initialize with another SE2 object.
         * 
         * @param State SE2 object. The matrix, covariance and augmented state will all be copied from it.
         */
        SE3(const SE3& State) : SE3(State(), State.cov(), State.aug()) {}
        
        /**
         * @brief Construct a new SE2 object from a tangent vector using the exponential operator.
         * 
         * @param xi Tangent vector of size (3 + 3*Columns + Augmented state size).
         * @param Cov Covariance of state. If not input, state is set as "certain" and covariance is not tracked.
         */
        SE3(const TangentVector& xi,
            const MatrixCov& Cov=MatrixCov::Zero(c,c));
        
        // TODO the -3 here is going to wreck havoc on dynamic types
        // Fix in python side too
        /**
         * @brief Construct a new SE3 object
         * 
         * @param R Rotational portion of the SE3 object.
         * @param xi Translational columns to input.
         * @param Cov Covariance of state. If not input, state is set as "certain" and covariance is not tracked.
         */
        SE3(const SO3<> R, const Eigen::Matrix<double,small_xi,1>& xi,
            const MatrixCov& Cov=MatrixCov::Zero(c,c));
        
        /**
         * @brief Construct a new SE3 object using exponential ooperator on angles and putting positions directly in.
         * 
         * @param w1 Rotaional component 1.
         * @param w2 Rotaional component 2.
         * @param w3 Rotaional component 3.
         * @param x X-position.
         * @param y Y-position.
         * @param z Z-position.
         * @param Cov Covariance of state. If not input, state is set as "certain" and covariance is not tracked.
         */
        SE3(double w1, double w2, double w3, double x, double y, double z,
            const MatrixCov& Cov=MatrixCov::Zero(c,c)){ 
                throw std::invalid_argument("Can't use this constructor with those templates");
        }

        /**
         * @brief Destroy the SE3 object
         * 
         */
        ~SE3() {}

        //------------ Getters
        /**
         * @brief Gets rotational component of the state.
         * 
         * @return SO2<> Rotational component of the state.
         */
        SO3<> R() const { 
            Eigen::Matrix3d R = State_.block(0,0,3,3);
            return SO3<>(R); 
        }

        /**
         * @brief Gets the ith positional column of the group.
         * 
         * @param idx Index of column to get, from 0 to C-1.
         * @return Eigen::Vector2d 
         */
        Eigen::Vector3d operator[](int idx) const {
            int curr_cols = State_.cols() - rotSize;
            if(idx >= curr_cols){
                throw std::out_of_range("Sliced out of range");
            }
            return State_.block(0,3+idx,3,1); 
        }

        /**
         * @brief Adds a column to the matrix state. Only usable if C = Eigen::Dynamic.
         * 
         * @param x Column to add in.
         * @param sigma Covariance of element. Only used if state is uncertain.
         */
        void addCol(const Eigen::Vector3d& x, const Eigen::Matrix3d& sigma=Eigen::Matrix3d::Identity());
        
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
        SE3 inverse() const;
        using LieGroup<SE3<C,A>,N,M,A>::Ad;
        using LieGroup<SE3<C,A>,N,M,A>::log;

        //------------ Group action
        /**
         * @brief Combine transformations. Augmented states are summed.
         * 
         * @param rhs Right hand element of multiplication.
         * @return Combined elements with same augmented size.
         */
        SE3 operator*(const SE3& rhs) const;

        //------------ Static Operators
        /**
         * @brief Move element in R^n to the Lie algebra.
         * 
         * @param xi Tangent vector
         * @return MatrixState Element of Lie algebra
         */
        static MatrixState wedge(const TangentVector& xi);

        /**
         * @brief Move an element from R^n -> algebra -> group
         * 
         * @param xi Tangent vector
         * @return Element of SE2
         */
        static SE3 exp(const TangentVector& xi);

        /**
         * @brief Move an element from group -> algebra -> R^n
         * 
         * @param g Group element
         * @return TangentVector 
         */
        static TangentVector log(const SE3& g);

        /**
         * @brief Compute the linear map Adjoint
         * 
         * @param g Element of SE2
         * @return Matrix of size state dimension x state dimension
         */
        static MatrixCov Ad(const SE3& g);

};

}

#include "SE3.tpp"

#endif // CLASS_SE3