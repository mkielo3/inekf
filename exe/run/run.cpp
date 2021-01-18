#include <Eigen/Dense>
#include <iostream>
#include "SE2_3_Bias/SE2_3_Bias.h"
#include "iekf/state.h"
#include "iekf/iekf.h"
#include "SE2_3_Bias/inertial.h"
#include "SE2_3_Bias/dvl.h"

int main(){
    Eigen::VectorXd z(3);
    z << 1, 1, 1;

    Eigen::VectorXd u(6);
    u << 0, 0, 0, 1, 1, 1;

    Eigen::VectorXd init(9);
    init << 0, 0, 0, 1, 1, 1, 2, 2, 2;

    SE2_3_Bias lie;
    State state(lie.ExpMountain(init), Eigen::MatrixXd::Identity(15,15));
    InEKF iekf(state);
    
    DVLSensor dvl;
    dvl.setNoise(0.5);
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