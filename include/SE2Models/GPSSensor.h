#ifndef GPSSENSOR
#define GPSSENSOR

#include <Eigen/Core>
#include <InEKF/Core>

namespace InEKF {

/**
 * @brief GPS Sensor for use in SE2 SLAM model.
 * 
 */
class GPSSensor : public MeasureModel<SE2<Eigen::Dynamic>> {
    public:
        using typename MeasureModel<SE2<Eigen::Dynamic>>::MatrixS;
        using typename MeasureModel<SE2<Eigen::Dynamic>>::MatrixH;
        using typename MeasureModel<SE2<Eigen::Dynamic>>::VectorV;
        using typename MeasureModel<SE2<Eigen::Dynamic>>::VectorB;

        /**
         * @brief Construct a new GPSSensor object
         * 
         * @param std The standard deviation of the measurement.
         */
        GPSSensor(double std=1);

        /**
         * @brief Destroy the GPSSensor object
         * 
         */
        ~GPSSensor(){ }

        /**
         * @brief Overriden from the base class. Needed to fill out H/z with correct number of columns based on number of landmarks in state.
         * 
         * @param z Measurement
         * @param state Current state estimate.
         * @return Processed measurement.
         */
        VectorB processZ(const Eigen::VectorXd& z, const SE2<Eigen::Dynamic>& state) override;
};

}

#endif // GPSSENSOR