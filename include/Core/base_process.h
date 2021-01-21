#ifndef BASE_PROCESS
#define BASE_PROCESS

#include <Eigen/Dense>
#include "Core/state.h"
#include "Core/base_lie.h"

namespace InEKF {

class ProcessModel {

    public:
        ProcessModel() {};
        virtual void f(const Eigen::VectorXd& u, double dt, State& state) = 0;
        virtual Eigen::MatrixXd MakePhi(const Eigen::VectorXd& u, double dt, const State& state) = 0;

        const Eigen::MatrixXd getQ() { return Q_; };

        LieGroup * lie_;

    protected:
        Eigen::MatrixXd Q_;

};

}

#endif // BASE_PROCESS