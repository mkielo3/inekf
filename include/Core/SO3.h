#ifndef CLASS_SO3
#define CLASS_SO3

#include <Eigen/Core>
#include "LieGroup.h"

namespace InEKF {

/**
 * @brief 3D rotational states, also known as the 3x3 special orthogonal group, SO(3).
 * 
 * @tparam A Number of augmented Euclidean states. Can be Eigen::Dynamic if desired. Defaults to 0.
 */
template<int A=0>
class SO3 : public LieGroup<SO3<A>,calcStateDim(3,0,A),3,A>{
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
        static constexpr int N = calcStateDim(rotSize,0,A);
        /**
         * @brief State will have matrix of size M x M
         * 
         */
        static constexpr int M = calcStateMtxSize(rotSize,0);

        using typename LieGroup<SO3<A>,N,M,A>::TangentVector;
        using typename LieGroup<SO3<A>,N,M,A>::MatrixCov;
        using typename LieGroup<SO3<A>,N,M,A>::MatrixState;
        using typename LieGroup<SO3<A>,N,M,A>::VectorAug;

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
        static constexpr int c = A == Eigen::Dynamic ? 3 : N;
        /**
         * @brief Handles defaults values of matrix sizes.
         * 
         */
        static constexpr int m = M;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
    private:
        using LieGroup<SO3<A>,N,M,A>::Cov_;
        using LieGroup<SO3<A>,N,M,A>::State_;
        using LieGroup<SO3<A>,N,M,A>::Aug_;
        using LieGroup<SO3<A>,N,M,A>::isUncertain;
        
        void verifySize();

    public:
        //------------ Constructors
        /**
         * @brief Construct SO3 object with all available options.
         * 
         * @param State A 2x2 Eigen matrix. Defaults to the identity.
         * @param Cov Covariance of state. If not input, state is set as "certain" and covariance is not tracked.
         * @param Aug Additional euclidean states if A != 0. Defaults to 0s. 
         */
        SO3(const MatrixState& State=MatrixState::Identity(), 
            const MatrixCov& Cov=MatrixCov::Zero(c,c),
            const VectorAug& Aug=VectorAug::Zero(a)) 
                : LieGroup<SO3<A>,N,M,A>(State, Cov, Aug) { verifySize(); }

        /**
         * @brief Copy constructor. Initialize with another SO3 object.
         * 
         * @param State SO3 object. The matrix, covariance and augmented state will all be copied from it.
         */
        SO3(const SO3& State) : SO3(State(), State.Cov(), State.Aug()) {}

        /**
         * @brief Construct a new SO3 object using angles and the matrix exponential.
         * 
         * @param w1 Angle 1.
         * @param w2 Angle 2.
         * @param w3 Angle 3.
         * @param Cov Covariance of state. If not input, state is set as "certain" and covariance is not tracked.
         * @param Aug Additional euclidean states if A != 0. Defaults to 0s. 
         */
        SO3(double w1, double w2, double w3, 
            const MatrixCov& Cov=MatrixCov::Zero(c,c),
            const VectorAug& Aug=VectorAug::Zero(a));

        /**
         * @brief Construct a new SO3 object from a tangent vector using the exponential operator.
         * 
         * @param xi Tangent vector of size (1 + Augmented state size).
         * @param Cov Covariance of state. If not input, state is set as "certain" and covariance is not tracked.
         */
        SO3(const TangentVector& xi, const MatrixCov& Cov=MatrixCov::Zero(c,c)) : SO3(xi(0), xi(1), xi(2), Cov, xi.tail(xi.size()-3)) {}

        /**
         * @brief Destroy the SO3 object
         * 
         */
        ~SO3() {}

        //------------ Getters
        /**
         * @brief Gets rotational component of the state. In the SO3 case, this is everything except the augmented Euclidean states and covariance.
         * 
         * @return SO3<> Rotational component of the state.
         */
        SO3<> R() const { return SO3<>(State_); }

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
         * @return Inverted matrix (transpose). Augmented portion and covariance is dropped.
         */
        SO3<A> inverse() const;
        using LieGroup<SO3<A>,N,M,A>::Ad;
        using LieGroup<SO3<A>,N,M,A>::log;

        //------------ Group action
        /**
         * @brief Combine rotations. Augmented states are summed.
         * 
         * @param rhs Right hand element of multiplication.
         * @return Combined elements with same augmented size.
         */
        SO3<A> operator*(const SO3<A>& rhs) const;

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
         * @return Element of SO3
         */
        static SO3 Exp(const TangentVector& xi);

        /**
         * @brief Move an element from group -> algebra -> R^n
         * 
         * @param g Group element
         * @return TangentVector 
         */
        static TangentVector Log(const SO3& g);

        /**
         * @brief Compute the linear map Adjoint
         * 
         * @param g Element of SO3
         * @return Matrix of size state dimension x state dimension
         */
        static MatrixCov Ad(const SO3& g);

};

}

#include "SO3.tpp"

#endif // CLASS_SO3