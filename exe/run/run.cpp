#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <boost/algorithm/string.hpp>
#include <vector>

#include "iekf/core.h"
#include "SE2_3_Bias/core.h"

int main(){
    // Set up initial state
    SE2_3_Bias lie;
    State state(lie, State::RIGHT);
    Eigen::Matrix3d R0;
    Eigen::Vector3d p0, v0;
    Eigen::VectorXd s(15);
    R0 << 0.00000, 0.99863, -0.05234,
         -0.99452, 0.00547, 0.10439,
          0.10453, 0.05205, 0.99316;
    v0 << 0, 0, -4.66134;
    p0 << -0.077, -0.02, -2.2082;
    s << 0.274156, 0.274156, 0.274156, 1.0, 1.0, 1.0, 0.01, 0.01, 0.01, 0.000025, 0.000025, 0.000025, 0.0025, 0.0025, 0.0025;
    state[0] = R0;
    state[1] = v0;
    state[2] = p0;
    state.setSigma( s.asDiagonal() );

    // Set up DVL
    Eigen::Vector3d dvl_p;
    Eigen::Matrix3d dvl_r;
    dvl_p << -0.17137, 0.00922, -0.33989;
    dvl_r << 0.000, -0.995, 0.105,
            0.999, 0.005, 0.052,
            -0.052, 0.104, 0.993;
    DVLSensor dvl(dvl_r, dvl_p);
    dvl.setNoise(.0101*2.6, .005*3.14/180*sqrt(200));

    // Set up Depth sensor
    DepthSensor depth;
    depth.setNoise(51 * 1/100 * 1/2);

    // Set up Inertial dynamics
    InertialProcess imu;
    imu.setGyroNoise( .005 *  3.14/180  * sqrt(200) );
    imu.setAccelNoise( 20 * pow(10, -6)/9.81 * sqrt(200) );
    imu.setGyroBiasNoise(0.001);
    imu.setAccelBiasNoise(0.001);

    // Set up IEKF
    InEKF iekf(state);
    iekf.setProcessModel(imu);
    iekf.addMeasureModel(dvl, "DVL");
    iekf.addMeasureModel(depth, "Depth");

    // Iterate through all of data!
    std::ifstream infile("data.txt");
    if(infile.fail()) std::cout << "Failed to open file" << std::endl;
    std::string line;
    Eigen::Matrix<double,6,1> imu_data = Eigen::Matrix<double,6,1>::Zero();
    Eigen::Matrix<double,3,1> dvl_data = Eigen::Matrix<double,3,1>::Zero();
    Eigen::Matrix<double,1,1> depth_data = Eigen::Matrix<double,1,1>::Zero();
    double dt = 0;
    int i = 0;
    while (getline(infile, line)){
        std::vector<std::string> measurement;
        boost::split(measurement,line,boost::is_any_of(" "));

        // // Handle measurements
        if (measurement[0].compare("IMU")==0){
            i+=1;
            std::cout << "Received IMU Data, propagating state\n";
            assert((measurement.size()-2) == 6);
            dt = atof(measurement[1].c_str()); 
            // Read in IMU data
            imu_data << atof(measurement[2].c_str()), 
                        atof(measurement[3].c_str()), 
                        atof(measurement[4].c_str()),
                        atof(measurement[5].c_str()),
                        atof(measurement[6].c_str()),
                        atof(measurement[7].c_str());
            State corrected = iekf.Predict(imu_data, dt);
        }

        else if (measurement[0].compare("DVL")==0){
            std::cout << "Received DVL Data, correcting state\n";
            assert((measurement.size()-2) == 3);
            dt = atof(measurement[1].c_str()); 
            // Read in DVL data
            dvl_data << atof(measurement[2].c_str()), 
                        atof(measurement[3].c_str()), 
                        atof(measurement[4].c_str());
            iekf.Update(dvl_data, "DVL");            
            std::cout << state.getAugment() << std::endl << std::endl;
        }

        // else if (measurement[0].compare("DEPTH")==0){
        //     std::cout << "Received Depth Data, correcting state\n";
        //     assert((measurement.size()-2) == 1);
        //     dt = atof(measurement[1].c_str()); 
        //     // Read in Depth data
        //     depth_data << atof(measurement[2].c_str());
        //     iekf.Update(depth_data, "Depth");   
        // }
    }
    
    std::cout << state.getMu() << std::endl;
    // State test_state(lie);
    // Eigen::VectorXd u(6);
    // u << 1, 1, 1, 1, 1, 12;

    // imu.f(u, .1, test_state);

    // std::cout << test_state.getMu() << std::endl;

    return 0;
}