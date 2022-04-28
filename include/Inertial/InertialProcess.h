#ifndef INERTIAL_PROCESS
#define INERTIAL_PROCESS

#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>
#include <InEKF/Core>

namespace Eigen{
    typedef Matrix<double,6,1> Vector6d;
}

namespace InEKF {

/**
 * @brief Inertial process model. Integrates IMU measurements and tracks biases. Requires "Imperfect InEKF" since biases 
 don't fit into Lie group structure.
 * 
 */
class InertialProcess : public ProcessModel<SE3<2,6>, Eigen::Matrix<double,6,1>> {

    private:
        const Eigen::Vector3d g_ = (Eigen::Vector3d() << 0,0,-9.81).finished();

    public:
        /**
        * @brief Construct a new Inertial Process object
        * 
        */
        InertialProcess();

        /**
         * @brief Destroy the Inertial Process object
         * 
         */
        ~InertialProcess(){}

        /**
         * @brief Overriden from base class. Integrates IMU measurements.
         * 
         * @param u Control
         * @param dt Delta time
         * @param state Current state
         * @return Integrated state
         */
        SE3<2,6> f(Eigen::Vector6d u, double dt, SE3<2,6> state) override;

        /**
         * @brief Overriden from base class. Since this is used in an "Imperfect InEKF", both left and right versions are slightly state dependent.
         * 
         * @param u Control
         * @param dt Delta time
         * @param state Current state estimate (shouldn't be needed unless doing an "Imperfect InEKF")
         * @param error Right or left error. Function should be implemented to handle both.
         * @return Phi
         */
        MatrixCov MakePhi(const Eigen::Vector6d& u, double dt, const SE3<2,6>& state, ERROR error) override;
        
        /**
         * @brief Set the gyro noise. Defaults to 0 if not set.
         * 
         * @param std Gyroscope standard deviation
         */
        void setGyroNoise(double std);

        /**
         * @brief Set the accelerometer noise. Defaults to 0 if not set.
         * 
         * @param std Accelerometer standard deviation
         */
        void setAccelNoise(double std);

        /**
         * @brief Set the gryo bias noise. Defaults to 0 if not set.
         * 
         * @param std Gyroscope bias standard deviation
         */
        void setGyroBiasNoise(double std);

        /**
         * @brief Set the accelerometer bias noise. Defaults to 0 if not set.
         * 
         * @param std Accelerometer bias standard deviation
         */
        void setAccelBiasNoise(double std);
        

};

}

#endif // INERTIAL_PROCESS