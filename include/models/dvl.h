#ifndef DVLSENSOR
#define DVLSENSOR

#include <Eigen/Dense>
#include "models/base_measure.h"
#include "lie/SE2_3.h"

class DVLSensor : public MeasureModel {
    
    public:
        DVLSensor();
        void Observe(Eigen::VectorXd& z, State& state);
        void setNoise(double std);

    private:
        SE2_3 lie;
};

#endif // DVLSENSOR