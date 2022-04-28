#ifndef DVLSENSOR
#define DVLSENSOR

#include <Eigen/Core>
#include <InEKF/Core>

namespace InEKF {

/**
 * @brief DVL sensor measurement model for use with inertial process model.
 * 
 */
class DVLSensor : public MeasureModel<SE3<2,6>> {
    
    public:
        using typename MeasureModel<SE3<2,6>>::MatrixS;
        using typename MeasureModel<SE3<2,6>>::MatrixH;
        using typename MeasureModel<SE3<2,6>>::VectorV;
        using typename MeasureModel<SE3<2,6>>::VectorB;

        /**
         * @brief Construct a new DVLSensor object. Assumes no rotation or translation between this and IMU frame.
         * 
         */
        DVLSensor();

        /**
         * @brief Construct a new DVLSensor object with offset from IMU frame.
         * 
         * @param dvlR 3x3 Rotation matrix encoding rotationf rom DVL to IMU frame.
         * @param dvlT 3x1 Vector of translation from IMU to DVL in IMU frame.
         */
        DVLSensor(Eigen::Matrix3d dvlR, Eigen::Vector3d dvlT);

        /**
         * @brief Construct a new DVLSensor object with offset from IMU frame.
         * 
         * @param dvlR SO3 object encoding rotationf rom DVL to IMU frame.
         * @param dvlT 3x1 Vector of translation from IMU to DVL in IMU frame.
         */
        DVLSensor(SO3<> dvlR, Eigen::Vector3d dvlT);

        /**
         * @brief Construct a new DVLSensor object with offset from IMU frame.
         * 
         * @param dvlH SE3 object encoding transformation from DVL to IMU frame.
         */
        DVLSensor(SE3<> dvlH);

        /**
         * @brief Destroy the DVLSensor object
         * 
         */
        ~DVLSensor(){ }

        /**
         * @brief Set the noise covariances.
         * 
         * @param std_dvl Standard deviation of DVL measurement.
         * @param std_imu Standard deviation of gyropscope measurement (needed b/c we transform frames).
         */
        void setNoise(double std_dvl, double std_imu);

        /**
         * @brief Overriden from base class. Takes in a 6 vector with DVL measurement as first 3 elements and IMU as last three 
         and converts DVL to IMU, then makes it the right size and passes it on.
         * 
         * @param z DVL/IMU measurement.
         * @param state Current state estimate.
         * @return Processed measurement.
         */
        VectorB processZ(const Eigen::VectorXd& z, const SE3<2,6>& state) override;

    private:
        Eigen::Matrix3d dvlT_;
        SO3<> dvlR_;
};

}

#endif // DVLSENSOR