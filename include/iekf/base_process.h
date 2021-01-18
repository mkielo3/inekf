#ifndef BASE_PROCESS
#define BASE_PROCESS

#include <Eigen/Dense>
#include "iekf/state.h"
#include "iekf/base_lie.h"

class ProcessModel {

    public:
        ProcessModel() {};
        virtual void f(Eigen::VectorXd u, double dt, State& state) = 0;
        virtual Eigen::MatrixXd MakePhi(Eigen::VectorXd u, double dt, State state) = 0;

        const Eigen::MatrixXd getQ() { return Q_; };

        LieGroup * lie_;

    protected:
        Eigen::MatrixXd Q_;

};

#endif // BASE_PROCESS