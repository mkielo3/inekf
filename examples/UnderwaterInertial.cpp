#include <Eigen/Core>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <boost/algorithm/string.hpp>
#include <vector>
#include <cmath>

#include <InEKF/Plot>
#include <InEKF/Core>
#include <InEKF/Inertial>

namespace plt = matplotlibcpp;

int main(){
    // Set up initial state
    Eigen::Matrix3d R0;
    R0 << 0.00000, 0.99863, -0.05234,
         -0.99452, 0.00547, 0.10439,
          0.10453, 0.05205, 0.99316;
    InEKF::SO3 Rot(R0);
    Eigen::Vector<double,12> xi;
    xi << 0, 0, -4.66134,       // velocity
        -0.077, -0.02, -2.2082, // position
        0,0,0,0,0,0;            // bias
    Eigen::Vector<double,15> s;
    s << 0.274156, 0.274156, 0.274156, 1.0, 1.0, 1.0, 0.01, 0.01, 0.01, 0.000025, 0.000025, 0.000025, 0.0025, 0.0025, 0.0025;
    InEKF::SE3<2,6> state(Rot, xi, s.asDiagonal());

    // Set up DVL
    Eigen::Matrix3d dvlR;
    dvlR << 0.000, -0.995, 0.105,
            0.999, 0.005, 0.052,
            -0.052, 0.104, 0.993;
    Eigen::Vector3d dvlT;
    dvlT << -0.17137, 0.00922, -0.33989;
    InEKF::DVLSensor dvl(dvlR, dvlT);
    dvl.setNoise(.0101*2.6, .005*(3.14/180)*sqrt(200.0));

    // Set up Depth sensor
    InEKF::DepthSensor depth(51.0 * (1.0/100) * (1.0/2));

    // Set up IEKF
    InEKF::InEKF<InEKF::InertialProcess> iekf(state, InEKF::RIGHT);
    iekf.addMeasureModel("DVL", &dvl);
    iekf.addMeasureModel("Depth", &depth);

    // Set up Inertial noise
    iekf.pModel->setGyroNoise( .005 *  (3.14/180)  * sqrt(200.0) );
    iekf.pModel->setAccelNoise( 20.0 * (pow(10, -6)/9.81) * sqrt(200.0) );
    iekf.pModel->setGyroBiasNoise(0.001);
    iekf.pModel->setAccelBiasNoise(0.001);



    // Iterate through all of data!
    int n = 3000; // holodeck_data goes to about ~3900
    int i = 0;
    std::ifstream infile("../data/holodeck_data.txt");
    if(infile.fail()) std::cout << "Failed to open file" << std::endl;
    std::string line;
    Eigen::Matrix<double,6,1> imu_data = Eigen::Matrix<double,6,1>::Zero();
    Eigen::Matrix<double,6,1> dvl_data = Eigen::Matrix<double,6,1>::Zero();
    Eigen::Matrix<double,1,1> depth_data = Eigen::Matrix<double,1,1>::Zero();
    Eigen::MatrixXd v(n,3), p(n,3), R(3*n,3);
    Eigen::MatrixXd v_result(n,3), p_result(n,3), R_result(3*n,3);

    double dt = 0;
    while (getline(infile, line)){
        std::vector<std::string> measurement;
        boost::split(measurement,line,boost::is_any_of(" "));

        // PREDICTION STEP
        if (measurement[0].compare("IMU")==0){
            std::cout << "Received IMU Data, propagating state\n";
            assert((measurement.size()-2) == 6);
            dt = atof(measurement[1].c_str()); 
            imu_data << atof(measurement[2].c_str()), 
                        atof(measurement[3].c_str()), 
                        atof(measurement[4].c_str()),
                        atof(measurement[5].c_str()),
                        atof(measurement[6].c_str()),
                        atof(measurement[7].c_str());
            state = iekf.Predict(imu_data, dt);
        }

        // UPDATE STEP
        else if (measurement[0].compare("DVL")==0){
            std::cout << "Received DVL Data, correcting state\n";
            assert((measurement.size()-2) == 3);
            dt = atof(measurement[1].c_str()); 
            dvl_data << atof(measurement[2].c_str()), 
                        atof(measurement[3].c_str()), 
                        atof(measurement[4].c_str()),
                        imu_data[0], imu_data[1], imu_data[2];
            state = iekf.Update(dvl_data, "DVL");            
        }
        else if (measurement[0].compare("DEPTH")==0){
            std::cout << "Received Depth Data, correcting state\n";
            assert((measurement.size()-2) == 1);
            dt = atof(measurement[1].c_str()); 
            depth_data << atof(measurement[2].c_str());
            state = iekf.Update(depth_data, "Depth");

            // DVL is last in list of data saved, so we save our state here
            R_result.block<3,3>(3*i,0) = state.R()();
            // save v in local frame
            v_result.row(i) = (state.R()().transpose() * state[0]).transpose();
            p_result.row(i) = state[1].transpose();
            i += 1;
            // std::cout << state.Aug() << std::endl;
        }

        // GROUND TRUTH TO COMPARE TO
        else if (measurement[0].compare("P")==0){
            std::cout << "Received Position Data\n";
            assert((measurement.size()-2) == 3);
            dt = atof(measurement[1].c_str()); 
            p.row(i) << atof(measurement[2].c_str()), atof(measurement[3].c_str()), atof(measurement[4].c_str());
        }
        else if (measurement[0].compare("V")==0){
            std::cout << "Received Velocity Data\n";
            assert((measurement.size()-2) == 3);
            dt = atof(measurement[1].c_str()); 
            v.row(i) << atof(measurement[2].c_str()), atof(measurement[3].c_str()), atof(measurement[4].c_str());
            // put v into local frame
            v.row(i) = ( R.block<3,3>(3*i,0).transpose() * v.row(i).transpose() ).transpose().eval();
        }
        else if (measurement[0].compare("R")==0){
            std::cout << "Received Orientation Data\n";
            assert((measurement.size()-2) == 9);
            dt = atof(measurement[1].c_str()); 
            R.block<3,3>(3*i,0) << atof(measurement[2].c_str()), atof(measurement[3].c_str()), atof(measurement[4].c_str()),
                                    atof(measurement[5].c_str()), atof(measurement[6].c_str()), atof(measurement[7].c_str()),
                                    atof(measurement[8].c_str()), atof(measurement[9].c_str()), atof(measurement[10].c_str());
        }

        if(i >= n){
            break;
        }
    }
    
    // Plot everything
    Eigen::VectorXd t = Eigen::VectorXd::LinSpaced(n, 0, n*dt);
    plt::figure_size(1200, 800);

    // Global position
    plt::subplot(3, 3, 1);
    plt::ylabel("Position");
    plt::title("X (global)");
    plt::plot(t, p.col(0), {{"label", "Actual"}});
    plt::plot(t, p_result.col(0), {{"label", "Prediction"}});
    plt::legend();
    plt::subplot(3, 3, 2);
    plt::title("Y (global)");
    plt::plot(t, p.col(1));
    plt::plot(t, p_result.col(1));
    plt::subplot(3, 3, 3);
    plt::title("Z (global)");
    plt::plot(t, p.col(2));
    plt::plot(t, p_result.col(2));

    // Local Velocity
    plt::subplot(3, 3, 4);
    plt::ylabel("Velocity");
    plt::title("X (local)");
    plt::plot(t, v.col(0));
    plt::plot(t, v_result.col(0));
    plt::subplot(3, 3, 5);
    plt::title("Y (local)");
    plt::plot(t, v.col(1));
    plt::plot(t, v_result.col(1));
    plt::subplot(3, 3, 6);
    plt::title("Z (local)");
    plt::plot(t, v.col(2));
    plt::plot(t, v_result.col(2));
    plt::tight_layout();

    // All the angles
    plt::subplot(3,3,7);
    plt::ylabel("Angles");
    plt::title("Pitch");
    Eigen::VectorXd angle(n);
    Eigen::VectorXd angle_result(n);
    for(int j=0;j<n;j++){
        angle[j] = -std::asin(R(3*j+2,0));
        angle_result[j] = -std::asin(R_result(3*j+2,0));
    }
    plt::plot(t, angle);
    plt::plot(t, angle_result);

    plt::subplot(3,3,8);
    plt::title("Roll");
    for(int j=0;j<n;j++){
        angle[j] = std::atan2(R(3*j+2,1), R(3*j+2,2));
        angle_result[j] = std::atan2(R_result(3*j+2,1), R_result(3*j+2,2));
    }
    plt::plot(t, angle);
    plt::plot(t, angle_result);

    plt::subplot(3,3,9);
    plt::title("Yaw");
    for(int j=0;j<n;j++){
        angle[j] = std::atan2(R(3*j+1,0), R(3*j,0));
        angle_result[j] = std::atan2(R_result(3*j+1,0), R_result(3*j,0));
    }
    plt::plot(t, angle);
    plt::plot(t, angle_result);

    plt::show();

    return 0;
}