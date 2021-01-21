#ifndef DEPTHSENSOR
#define DEPTHSENSOR

#include <Eigen/Dense>
#include "Core/base_measure.h"
#include "SE2_3_Bias/SE2_3_Bias.h"

#include <iostream>

namespace InEKF {

class DepthSensor : public MeasureModel {
    
    public:
        DepthSensor();
        ~DepthSensor(){ delete lie_; }
        void Observe(const Eigen::VectorXd& z, const State& state);
        void setNoise(double std);
};

}

#endif // DEPTHSENSOR