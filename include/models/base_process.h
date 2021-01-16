#ifndef BASE_PROCESS
#define BASE_PROCESS

#include <Eigen/Dense>
#include "iekf/state.h"
#include "lie/base.h"

class ProcessModel {

    public:
        ProcessModel() {};
        virtual void f(Eigen::VectorXd u, double dt, State& state) = 0;
        virtual Eigen::MatrixXd MakePhi(Eigen::VectorXd u, double dt, State state) = 0;

};

#endif // BASE_PROCESS