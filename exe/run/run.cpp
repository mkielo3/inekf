#include <Eigen/Dense>
#include <iostream>
#include "lie/SE2_3.h"
#include "iekf/state.h"
#include "models/inertial.h"
#include "models/dvl.h"

int main(){
    Eigen::VectorXd z(3);
    z << 1, 1, 1;

    Eigen::VectorXd init(9);
    init << 0, 0, 0, 1, 1, 1, 2, 2, 2;

    SE2_3 lie;
    State state(lie.ExpMountain(init), Eigen::MatrixXd::Identity(15,15));
    DVLSensor dvl;

    dvl.setNoise(0.1);
    dvl.Observe(z, state);

    std::cout << dvl.getV() << std::endl << std::endl;
    std::cout << dvl.getH() << std::endl << std::endl;
    std::cout << dvl.getSinv() << std::endl << std::endl;

    return 0;
}