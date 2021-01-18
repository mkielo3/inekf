#ifndef INERTIAL_PROCESS
#define INERTIAL_PROCESS

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include "iekf/base_process.h"
#include "iekf/state.h"
#include "SE2_3_Bias/SE2_3_Bias.h"

class InertialProcess : public ProcessModel {

    public:
        InertialProcess();
        ~InertialProcess(){ delete lie_; }
        void f(Eigen::VectorXd u, double dt, State& state);
        Eigen::MatrixXd MakePhi(Eigen::VectorXd u, double dt, State state, ERROR error);
        
        void setGyroNoise(double std);
        void setAccelNoise(double std);
        void setGyroBiasNoise(double std);
        void setAccelBiasNoise(double std);
        
    private:
        const Eigen::Vector3d g_;

};

#endif // INERTIAL_PROCESS