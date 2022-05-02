#ifndef DEPTHSENSOR
#define DEPTHSENSOR

#include <Eigen/Core>
#include <InEKF/Core>

namespace InEKF {

/**
 * @brief Pressure/Depth sensor measurement model for use with inertial process model. Uses 
    pseudo-measurements to fit into a left invariant measurement model.
 * 
 */
class DepthSensor : public MeasureModel<SE3<2,6>> {
    
    public:
        using typename MeasureModel<SE3<2,6>>::MatrixS;
        using typename MeasureModel<SE3<2,6>>::MatrixH;
        using typename MeasureModel<SE3<2,6>>::VectorV;
        using typename MeasureModel<SE3<2,6>>::VectorB;

        /**
         * @brief Construct a new Depth Sensor object
         * 
         * @param std The standard deviation of the measurement.
         */
        DepthSensor(double std=1);

        /**
         * @brief Destroy the Depth Sensor object
         * 
         */
        ~DepthSensor(){ }

        /**
         * @brief Overriden from the base class. Inserts psuedo measurements for the x and y value to fit the invariant measurement.
         * 
         * @param z Measurement
         * @param state Current state estimate.
         * @return Processed measurement.
         */
        VectorB processZ(const Eigen::VectorXd& z, const SE3<2,6>& state) override;

        /**
         * @brief Overriden from base class. Calculate inverse of measurement noise S, using the Woodbury Matrix Identity
         * 
         * @param state Current state estimate.
         * @return Inverse of measurement noise. 
         */
        MatrixS calcSInverse(const SE3<2,6>& state) override;

        /**
         * @brief Set the measurement noise
         * 
         * @param std The standard deviation of the measurement.
         */
        void setNoise(double std);
};

}

#endif // DEPTHSENSOR