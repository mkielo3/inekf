#ifndef BASE_LIE
#define BASE_LIE

#include <Eigen/Core>
#include <string>
#include <ostream>

namespace InEKF {

enum ERROR { LEFT, RIGHT };

constexpr int calcStateDim(int rotMtxSize, int C, int A){
    if(A == Eigen::Dynamic || C == Eigen::Dynamic){
        return Eigen::Dynamic;
    }
    else{
        return rotMtxSize*(rotMtxSize-1)/2 + rotMtxSize *C + A;
    }
}
constexpr int calcStateMtxSize(int rotMtxSize, int C){
    if (C == Eigen::Dynamic){
        return Eigen::Dynamic;
    }
    else{
        return rotMtxSize + C;
    }
}


/**
 * @brief Base Lie Group Class
 * 
 * @tparam Class Class that is inheriting from it. Allows for better polymorphism
 * @tparam N Group dimension
 * @tparam M Lie Group matrix size
 * @tparam A Augmented Euclidean state size
 */
template  <class Class, int N, int M, int A>
class LieGroup{

    public:
        /**
         * @brief A tangent vector has size Nx1, where N is the state dimension.
         * 
         */
        typedef Eigen::Matrix<double, N, 1> TangentVector;
        /**
         * @brief The covariance matrix has size NxN, where N is the state dimension.
         * 
         */
        typedef Eigen::Matrix<double, N, N> MatrixCov;
        /**
         * @brief A group element is a matrix of size MxM.
         * 
         */
        typedef Eigen::Matrix<double, M, M> MatrixState;
        /**
         * @brief Vector of additional Euclidean states, of size Ax1.
         * 
         */
        typedef Eigen::Matrix<double, A, 1> VectorAug;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    protected:
        /**
        * @brief Holds the actual group matrix.
        * 
        */
        MatrixState State_;

        /**
         * @brief Covariance of state. Defaults to 0s/certain if not set.
         * 
         */
        MatrixCov Cov_;

        /**
         * @brief Augmented Euclidean state that can be tracked.
         * 
         */
        VectorAug Aug_;

        /**
         * @brief Whether uncertainty (covariance) is being tracked.
         * 
         */
        bool isUncertain;

    public:
        /**
        * @brief Construct a new Lie Group object. Default Contructor.
        * 
        */
        LieGroup() {}

        /**
         * @brief Construct a new Lie Group object
         * 
         * @param State Group element
         * @param Cov Covariance of state. If not input, state is set as "certain" and covariance is not tracked.
         * @param Aug Additional euclidean states if A != 0. Defaults to 0s. 
         */
        LieGroup(MatrixState State, MatrixCov Cov, VectorAug Aug) 
            : State_(State), Cov_(Cov), Aug_(Aug), isUncertain(!Cov.isZero()) {}

        /**
         * @brief Construct a new Lie Group object
         * 
         * @param Cov Covariance of state. If not input, state is set as "certain" and covariance is not tracked.
         * @param Aug Additional euclidean states if A != 0. Defaults to 0s. 
         */
        LieGroup(MatrixCov Cov, VectorAug Aug) 
            : Cov_(Cov), Aug_(Aug), isUncertain(!Cov.isZero()) {}

        /**
         * @brief Construct a new Lie Group object
         * 
         * @param Cov Covariance of state. If not input, state is set as "certain" and covariance is not tracked.
         */
        LieGroup(MatrixCov Cov) 
            : Cov_(Cov), isUncertain(!Cov.isZero()) {}

        /**
         * @brief Destroy the Lie Group object
         * 
         */
        virtual ~LieGroup() {};

        //------------ Getters
        /**
         * @brief Returns whether object is uncertain, ie if it has a covariance.
         * 
         * @return true 
         * @return false 
         */
        bool Uncertain() const { return isUncertain; }

        /**
         * @brief Get covariance of group element.
         * 
         * @return const MatrixCov& 
         */
        const MatrixCov& Cov() const { return Cov_; }

        /**
         * @brief Get additional Euclidean state of object.
         * 
         * @return const VectorAug& 
         */
        const VectorAug& Aug() const { return Aug_; }

        /**
         * @brief Get actual group element.
         * 
         * @return const MatrixState& 
         */
        const MatrixState& operator()() const { return State_; }
        
        //------------ Setters
        /**
         * @brief Set the state covariance
         * 
         * @param Cov Covariance matrix.
         */
        void setCov(const MatrixCov& Cov) { Cov_ = Cov; };

        /**
         * @brief Set the additional augmented state
         * 
         * @param Aug Augmented state vector.
         */
        void setAug(const VectorAug& Aug) { Aug_ = Aug; };

        /**
         * @brief Set the group element.
         * 
         * @param State matrix Lie group element.
         */
        void setState(const MatrixState& State) { State_ = State; }

        // helper to automatically cast things
        /**
         * @brief Cast LieGroup object to object that is inheriting from it.
         * 
         * @return const Class& 
         */
        const Class & derived() const{
            return static_cast<const Class&>(*this);
        }

        //------------ Self operations
        /**
         * @brief Invert group element.
         * 
         * @return Inverted group element. Augmented portion and covariance is dropped.
         */
        Class inverse() const {
            return derived().inverse();
        }
        
        /**
         * @brief Move this element from group -> algebra -> R^n
         * 
         * @return TangentVector 
         */
        TangentVector log() const {
            return Class::Log(derived());
        }
        /**
         * @brief Get adjoint of group element.
         * 
         * @return MatrixCov 
         */
        MatrixCov Ad() const{
            return Class::Ad(derived());
        }

        //------------ Group action
        /**
         * @brief Multiply group elements.
         * 
         * @param g Group element.
         * @return Class 
         */
        Class compose(const Class& g) const {
            return derived() * g;
        }

        //------------ Static Operators
        /**
         * @brief Move element in R^n to the Lie algebra.
         * 
         * @param xi Tangent vector
         * @return MatrixState Element of Lie algebra
         */        
        static MatrixState Wedge(const TangentVector& xi){
            return Class::Wedge(xi);
        }

        /**
         * @brief Move an element from R^n -> algebra -> group
         * 
         * @param xi Tangent vector
         * @return Element of SO3
         */
        static Class Exp(const TangentVector& xi){
            return Class::Exp(xi);
        }

        /**
         * @brief Move an element from group -> algebra -> R^n
         * 
         * @param g Group element
         * @return TangentVector 
         */
        static TangentVector Log(const Class& g){
            return Class::Log(g);
        }

        /**
         * @brief Compute the linear map Adjoint
         * 
         * @param g Element of SO3
         * @return Matrix of size state dimension x state dimension
         */
        static MatrixCov Ad(const Class& g){
            return Class::Ad(g);
        }

        /**
         * @brief Convert group element to string. If uncertain print covariance as well. If has augmented state, print that as well.
         * 
         * @return std::string 
         */
        std::string toString() const{
            std::ostringstream os;

            os << "Matrix\n" << State_;
            if(isUncertain) os << "\nSigma\n" << Cov_;
            if(A != 0) os << "\nAug\n" << Aug_;

            return os.str();
        }

};

}

template <class Class, int N, int M, int A>
std::ostream& operator<<(std::ostream& os, const InEKF::LieGroup<Class,N,M,A>& rhs)  
{
    os << "Matrix\n" << rhs();
    if(rhs.Uncertain()) os << "\nSigma\n" << rhs.Cov();
    if(A != 0) os << "\nAug\n" << rhs.Aug();
    return os;
}

#endif // BASE_LIE