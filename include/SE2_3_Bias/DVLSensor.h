#ifndef DVLSENSOR
#define DVLSENSOR

#include <Eigen/Dense>
#include "Core/MeasureModel.h"
#include "SE2_3_Bias/SE2_3_Bias.h"

namespace InEKF {

class DVLSensor : public MeasureModel {
    
    public:
        DVLSensor();
        DVLSensor(Eigen::Matrix3d dvl_r, Eigen::Vector3d dvl_p);
        ~DVLSensor(){ delete lie_; }
        void Observe(const Eigen::VectorXd& z, const State& state);
        void setNoise(double std_dvl, double std_imu);

    private:
        Eigen::Matrix3d dvl_p;
        Eigen::Matrix3d dvl_r;
};

}

#endif // DVLSENSOR