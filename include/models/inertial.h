#ifndef INERTIAL_PROCESS
#define INERTIAL_PROCESS

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include "models/base_process.h"
#include "iekf/state.h"
#include "lie/SE2_3.h"

class InertialProcess : public ProcessModel {

    public:
        InertialProcess();
        void f(Eigen::VectorXd u, double dt, State& state);
        Eigen::MatrixXd MakePhi(Eigen::VectorXd u, double dt, State state);
        
    private:
        SE2_3 lie_;
        const Eigen::Vector3d g_;

};

#endif // INERTIAL_PROCESS