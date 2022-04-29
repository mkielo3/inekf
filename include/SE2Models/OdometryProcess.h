#ifndef ODOMETRY_PROCESS
#define ODOMETRY_PROCESS

#include <Eigen/Core>

#include "InEKF/Core"

namespace InEKF {

/**
 * @brief Odometry process model with fixed number of columns.
 * 
 */
class OdometryProcess : public ProcessModel<SE2<>, SE2<>> {

    public:
        /**
        * @brief Construct a new Odometry Process object
        * 
        */
        OdometryProcess(){}

        /**
         * @brief Construct a new Odometry Process object and set corresponding covariances.
         * 
         * @param theta_cov Standard deviation of rotation between timesteps.
         * @param x_cov Standard deviation of x between timesteps.
         * @param y_cov Standard deviation of y between timesteps.
         */
        OdometryProcess(float theta_cov, float x_cov, float y_cov){
            Eigen::Vector3d q;
            q << theta_cov, x_cov, y_cov;
            setQ(q);
        }

        /**
         * @brief Construct a new Odometry Process object. Set Q from vector. 
         * 
         * @param q Vector that becomes diagonal of Q.
         */
        OdometryProcess(Eigen::Vector3d q){ setQ(q); }

        /**
         * @brief Construct a new Odometry Process object. Set Q from matrix.
         * 
         * @param q Matrix that is set as Q.
         */
        OdometryProcess(Eigen::Matrix3d q){ setQ(q); }

        /**
         * @brief Destroy the Odometry Process object
         * 
         */
        ~OdometryProcess(){}
        
        /**
         * @brief Overriden from base class. Propagates the model \f$X_{t+1} = XU\f$
         * 
         * @param u Control
         * @param dt Delta time
         * @param state Current state
         * @return Updated state estimate
         */
        SE2<> f(SE2<> u, double dt, SE2<> state) override;

        /**
         * @brief Overriden from base class. If right, this is the identity. If left, it's the adjoint of U.
         * 
         * @param u Control
         * @param dt Delta time
         * @param state Current state estimate (shouldn't be needed unless doing an "Imperfect InEKF")
         * @param error Right or left error. Function should be implemented to handle both.
         * @return Phi
         */
        MatrixCov makePhi(const SE2<>& u, double dt, const SE2<>& state, ERROR error) override;
        
        /**
         * @brief Set Q from vector.
         * 
         * @param q Vector that becomes diagonal of Q.
         */
        void setQ(Eigen::Vector3d q) { this->Q_ = q.asDiagonal(); }

        /**
         * @brief Set Q from matrix.
         * 
         * @param q Matrix that is set as Q.
         */
        void setQ(Eigen::Matrix3d q) { this->Q_ = q; }

        /**
         * @brief Set Q from scalar.
         * 
         * @param q Scalar that becomes diagonal of Q
         */
        void setQ(double q) { this->Q_ = q*Eigen::Matrix3d::Identity(); }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


}

#endif // ODOMETRY_PROCESS