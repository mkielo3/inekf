#ifndef CLASS_SO2
#define CLASS_SO2

#include <Eigen/Core>
#include "LieGroup.h"

namespace InEKF {

template<int A=0>
class SO2 : public LieGroup<SO2<A>,calcStateDim(2,0,A),2,A>{
    public:
        static constexpr int rotSize = 2;
        static constexpr int N = calcStateDim(rotSize,0,A);
        static constexpr int M = calcStateMtxSize(rotSize,0);

        using typename LieGroup<SO2<A>,N,M,A>::TangentVector;
        using typename LieGroup<SO2<A>,N,M,A>::MatrixCov;
        using typename LieGroup<SO2<A>,N,M,A>::MatrixState;
        using typename LieGroup<SO2<A>,N,M,A>::VectorAug;

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
        static constexpr int c = A == Eigen::Dynamic ? 1 : N;
        /**
         * @brief Handles defaults values of matrix sizes.
         * 
         */
        static constexpr int m = M;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
        using LieGroup<SO2<A>,N,M,A>::Cov_;
        using LieGroup<SO2<A>,N,M,A>::State_;
        using LieGroup<SO2<A>,N,M,A>::Aug_;
        using LieGroup<SO2<A>,N,M,A>::isUncertain;
        
        /**
         * @brief Used to make sure covariance dimension and state dimension line up.
         * 
         */
        void verifySize();

    public:
        //------------ Constructors
        /**
         * @brief Construct SO2 object with all available options.
         * 
         * @param State An element of SO2. Defaults to the identity.
         * @param Cov Covariance of state. Defaults to no uncertainty.
         * @param Aug Additional euclidean states if A != 0. Defaults to 0s. 
         */
        SO2(const MatrixState& State=MatrixState::Identity(), 
            const MatrixCov& Cov=MatrixCov::Zero(c,c),
            const VectorAug& Aug=VectorAug::Zero(a)) 
                : LieGroup<SO2<A>,N,M,A>(State, Cov, Aug) { verifySize(); }

        /**
         * @brief Copy constructor. Initialize with another SO2 object.
         * 
         * @param State Element of SO2. The matrix, covariance and augmented state will all be copied from it.
         */
        SO2(const SO2& State) : SO2(State(), State.Cov(), State.Aug()) {}

        /**
         * @brief Construct a new SO2 object using an angle.
         * 
         * @param theta Angle of rotation matrix in degrees.
         * @param Cov Covariance of state. Defaults to no uncertainty.
         * @param Aug Additional euclidean states if A != 0. Defaults to 0s. 
         */
        SO2(double theta, 
            const MatrixCov& Cov=MatrixCov::Zero(c,c),
            const VectorAug& Aug=VectorAug::Zero(a)) 
                : LieGroup<SO2<A>,N,M,A>(Cov, Aug) {
            State_ << cos(theta), -sin(theta),
                    sin(theta), cos(theta);
            verifySize();
        }

        /**
         * @brief Construct a new SO2 object from a tangent vector using the exponential operator.
         * 
         * @param xi Tangent vector of size 1 + Augmented state size.
         * @param Cov Covariance of state. Defaults to no uncertainty.
         */
        SO2(const TangentVector& xi, const MatrixCov& Cov=MatrixCov::Zero(c,c)) : SO2(xi(0), Cov, xi.tail(xi.size()-1)) {}

        ~SO2() {}

        //------------ Getters
        /**
         * @brief Gets rotational component of the state. In the SO2 case, this is everything except the augmented Euclidean states and covariance.
         * 
         * @return SO2<> Rotational component of the state.
         */
        SO2<> R() const { return SO2<>(State_); }

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
         * @return Inverted matrix (transpose). Augmented portion is left alone.
         */
        SO2<A> inverse() const;

        using LieGroup<SO2<A>,N,M,A>::Ad;
        using LieGroup<SO2<A>,N,M,A>::log;

        //------------ Group action
        /**
         * @brief Combine rotations. Augmented states are summed.
         * 
         * @param rhs Right hand element of multiplication.
         * @return Combined elements with same augmented size.
         */
        SO2<A> operator*(const SO2<A>& rhs) const;

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
         * @return Element of SO2
         */
        static SO2 Exp(const TangentVector& xi);

        /**
         * @brief Move an element from group -> algebra -> R^n
         * 
         * @param g Group element
         * @return TangentVector 
         */
        static TangentVector Log(const SO2& g);

        /**
         * @brief Compute the linear map Adjoint
         * 
         * @param g Element of SO2
         * @return Matrix of size state dimension x state dimension
         */
        static MatrixCov Ad(const SO2& g);

};

}

#include "SO2.tpp"

#endif // CLASS_SO2