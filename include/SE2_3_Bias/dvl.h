#ifndef DVLSENSOR
#define DVLSENSOR

#include <Eigen/Dense>
#include "iekf/base_measure.h"
#include "SE2_3_Bias/SE2_3_Bias.h"

class DVLSensor : public MeasureModel {
    
    public:
        DVLSensor();
        void Observe(Eigen::VectorXd& z, State& state);
        void setNoise(double std);

    private:
        SE2_3_Bias lie;
};

#endif // DVLSENSOR