#ifndef INERTIAL_PROCESS
#define INERTIAL_PROCESS

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include "Core/ProcessModel.h"
#include "Core/State.h"
#include "SE2_3_Bias/SE2_3_Bias.h"

namespace InEKF {

class InertialProcess : public ProcessModel {

    public:
        InertialProcess();
        ~InertialProcess(){ delete lie_; }
        void f(const Eigen::VectorXd& u, double dt, State& state);
        Eigen::MatrixXd MakePhi(const Eigen::VectorXd& u, double dt, const State& state);
        
        void setGyroNoise(double std);
        void setAccelNoise(double std);
        void setGyroBiasNoise(double std);
        void setAccelBiasNoise(double std);
        
    private:
        const Eigen::Vector3d g_;

};

}

#endif // INERTIAL_PROCESS