#include <Eigen/Dense>
#include <iostream>
#include "iekf/core.h"
#include "SE2_3_Bias/core.h"

int main(){
    InEKF::ERROR test;

    Eigen::VectorXd z(3);
    z << 1, 2, 3;

    Eigen::VectorXd u(6);
    u << 2, 2, 2, 1, 1, 1;

    Eigen::VectorXd init(9);
    init << 0, 0, 0, 1, 1, 1, 2, 2, 2;

    SE2_3_Bias lie;
    State state(lie.ExpMountain(init), Eigen::MatrixXd::Identity(15,15));
    InEKF iekf(state);
    
    DVLSensor dvl(Eigen::Matrix3d::Identity(), z);
    dvl.setNoise(0.5, 0.1);
    InertialProcess imu;
    imu.setGyroNoise(0.1);
    imu.setAccelNoise(0.1);
    imu.setGyroBiasNoise(0.2);
    imu.setAccelBiasNoise(0.2);

    iekf.setProcessModel(imu);
    iekf.addMeasureModel(dvl, "DVL");
    
    std::cout << state.getSigma() << std::endl;
    State updated = iekf.Update(u, .1);
    std::cout << updated.getSigma() << std::endl;

    State corrected = iekf.Correct(z, "DVL");
    std::cout << corrected.getSigma() << std::endl;

    return 0;
}