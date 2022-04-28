#ifndef LANDMARKSENSOR
#define LANDMARKSENSOR

#include <Eigen/Core>
#include <InEKF/Core>

namespace InEKF {

/**
 * @brief Landmark sensor used in SLAM on SE2
 * 
 */
class LandmarkSensor : public MeasureModel<SE2<Eigen::Dynamic>> {

    public:
        using typename MeasureModel<SE2<Eigen::Dynamic>>::MatrixS;
        using typename MeasureModel<SE2<Eigen::Dynamic>>::MatrixH;
        using typename MeasureModel<SE2<Eigen::Dynamic>>::VectorV;
        using typename MeasureModel<SE2<Eigen::Dynamic>>::VectorB;

    private:
        Eigen::VectorXd b_;
        MatrixS M_rb;
        Eigen::Matrix<double,2,4> HSmall;
        int lmIdx;
        ERROR filterError;
        
    public:
        /**
        * @brief Construct a new Landmark Sensor object
        * 
        * @param std_r Range measurement standard deviation
        * @param std_b Bearing measurement standard deviation
        */
        LandmarkSensor(double std_r, double std_b);

        /**
         * @brief Destroy the Landmark Sensor object
         * 
         */
        ~LandmarkSensor(){ }

        /**
         * @brief Sets H based on what landmark was recently seen.
         * 
         * @param idx Index of landmark recently seen.
         * @param state Current state estimate. Used for # of landmarks.
         */
        void sawLandmark(int idx, const SE2<Eigen::Dynamic>& state);

        // used for data assocation, to reduce calls to C++
        /**
         * @brief Calculates Mahalanobis distance of having seen a certain landmark. Used for data association.
         * 
         * @param z Range and bearing measurement
         * @param state Current state estimate
         * @return Mahalanobis distance
         */
        double calcMahDist(const Eigen::VectorXd& z, const SE2<Eigen::Dynamic>& state);

        /**
         * @brief Overriden from base class. Converts r,b -> x,y coordinates and shifts measurement covariance. Then fills out z accordingly.
         * 
         * @param z Measurement
         * @param state Current state estimate.
         * @return Processed measurement.
         */
        VectorB processZ(const Eigen::VectorXd& z, const SE2<Eigen::Dynamic>& state) override;
        
        /**
         * @brief Overriden from base class. If using RInEKF, takes advantage of sparsity of H to shrink matrix multiplication. 
         Otherwise, operates identically to base class.
         * 
         * @param state Current state estimate.
         * @return Inverse of measurement noise. 
         */
        MatrixS calcSInverse(const SE2<Eigen::Dynamic>& state) override;
        
        /**
         * @brief Overriden from base class. Saves filter error for later use, then calls base class.
         * @param state Current state estimate.
         * @param iekfERROR Type of filter error.
         * @return H_error_ 
         */
        MatrixH makeHError(const SE2<Eigen::Dynamic>& state, ERROR iekfERROR) override {
            filterError = iekfERROR;
            return MeasureModel<SE2<Eigen::Dynamic>>::makeHError(state, iekfERROR);
        }

};

}

#endif // LANDMARKSENSOR