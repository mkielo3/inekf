#ifndef ODOMETRY_PROCESS_DYNAMIC
#define ODOMETRY_PROCESS_DYNAMIC

#include <Eigen/Core>

#include "InEKF/Core"

namespace InEKF {

/**
 * @brief Odometry process model with variable number of columns, for use in SLAM on SE2.
 * 
 */
class OdometryProcessDynamic : public ProcessModel<SE2<Eigen::Dynamic>, SE2<>> {

    public:
        /**
        * @brief Construct a new Odometry Process Dynamic object
        * 
        */
        OdometryProcessDynamic(){}

        /**
         * @brief Construct a new Odometry Process Dynamic object and set corresponding covariances.
         * 
         * @param theta_cov Standard deviation of rotation between timesteps.
         * @param x_cov Standard deviation of x between timesteps.
         * @param y_cov Standard deviation of y between timesteps.
         */
        OdometryProcessDynamic(float theta_cov, float x_cov, float y_cov){
            Eigen::Vector3d q;
            q << theta_cov, x_cov, y_cov;
            setQ(q);
        }

        /**
         * @brief Construct a new Odometry Process Dynamic object. Set Q from vector. 
         * 
         * @param q Vector that becomes diagonal of Q.
         */
        OdometryProcessDynamic(Eigen::Vector3d q){ setQ(q); }

        /**
         * @brief Construct a new Odometry Process Dynamic object. Set Q from matrix.
         * 
         * @param q Matrix that is set as Q.
         */
        OdometryProcessDynamic(Eigen::Matrix3d q){ setQ(q); }

        /**
         * @brief Destroy the Odometry Process Dynamic object
         * 
         */
        ~OdometryProcessDynamic(){}
        
        /**
         * @brief Overriden from base class. Propagates the model \f$X_{t+1} = XU\f$. Landmarks are left as is.
         * 
         * @param u Control
         * @param dt Delta time
         * @param state Current state
         * @return Updated state estimate
         */
        SE2<Eigen::Dynamic> f(SE2<> u, double dt, SE2<Eigen::Dynamic> state) override;
        
        /**
         * @brief Overriden from base class. If right, this is the identity. If left, it's the adjoint of U. 
         Landmark elements are the identity in both versions of Phi.
         * 
         * @param u Control
         * @param dt Delta time
         * @param state Current state estimate (shouldn't be needed unless doing an "Imperfect InEKF")
         * @param error Right or left error. Function should be implemented to handle both.
         * @return Phi
         */
        MatrixCov makePhi(const SE2<>& u, double dt, const SE2<Eigen::Dynamic>& state, ERROR error) override;
        
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

#endif // ODOMETRY_PROCESS_DYNAMIC