#ifndef INERTIAL_PROCESS
#define INERTIAL_PROCESS

#include <Eigen/Dense>
#include "iekf/state.h"
#include "models/base_process.h"

class InertialProcess : public ProcessModelBase {

    public:
        InertialProcess() {};
        State f(Eigen::VectorXd u, State state);
        Eigen::MatrixXd MakeA(Eigen::VectorXd u, State state);
        
    private:
        

};

#endif // INERTIAL_PROCESS