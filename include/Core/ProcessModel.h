#ifndef BASE_PROCESS
#define BASE_PROCESS

#include <Eigen/Core>
#include "Core/LieGroup.h"

namespace InEKF {

/**
 * @brief Base class process model.
 * 
 * @tparam Group State's group that is being tracked.
 * @tparam U Form of control. Can be either a group, or an Eigen::Matrix<double,n,1>
 */
template <class Group, class U>
class ProcessModel {

    public:
        /**
         * @brief The covariance matrix has size NxN, where N is the state dimension.
        * 
        */
        typedef typename Group::MatrixCov MatrixCov;

        /**
         * @brief A group element is a matrix of size MxM.
         * 
         */
        typedef typename Group::MatrixState MatrixState;

        /**
         * @brief Renaming of Group template, used by the InEKF
         * 
         */
        typedef Group myGroup;

        /**
         * @brief Renaming of U template, used by the InEKF
         * 
         */
        typedef U myU;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    protected:
        /**
        * @brief Process model covariance
        * 
        */
        MatrixCov Q_;

    public:
        /**
        * @brief Construct a new Process Model object
        * 
        */
        ProcessModel() {};

        /**
         * @brief Propagates state forward one timestep. Must be overriden, has no implementation.
         * 
         * @param u Control
         * @param dt Delta time
         * @param state Current state
         * @return Updated state estimate
         */
        virtual Group f(U u, double dt, Group state) { throw std::invalid_argument("f needs to be overriden."); return state; }

        /**
         * @brief Make a discrete time linearized process model matrix, with \f$\Phi = \exp(A\Delta t) \f$. Must be overriden, has no implementation.
         * 
         * @param u Control
         * @param dt Delta time
         * @param state Current state estimate (shouldn't be needed unless doing an "Imperfect InEKF")
         * @param error Right or left error. Function should be implemented to handle both.
         * @return Phi
         */
        virtual MatrixCov makePhi(const U& u, double dt, const Group& state, ERROR error) { throw std::invalid_argument("f needs to be overriden."); return MatrixCov::Identity(1,1); }

        /**
         * @brief Get process model covariance
         * 
         * @return Q 
         */
        MatrixCov getQ() const { return Q_; };

        /**
         * @brief Set process model covariance
         * 
         * @param Q 
         */
        void setQ(MatrixCov Q) { Q_ = Q; };

};

}

#endif // BASE_PROCESS