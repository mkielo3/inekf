#include <Eigen/Dense>
#include <iostream>
#include "lie/SE2_3.h"
#include "iekf/state.h"
#include "models/inertial.h"

int main(){
    Eigen::VectorXd u(6);
    u << 0, 0, 0, 1, 1, 11;

    Eigen::VectorXd init(9);
    init << 0, 0, 0, 1, 1, 1, 2, 2, 2;

    SE2_3 lie;
    State state(lie.ExpMountain(init));
    InertialProcess p;

    p.f(u, .1, state);

    std::cout << lie.ExpMountain(init) << std::endl;
    std::cout << state.getVelocity() << std::endl;

    return 0;
}