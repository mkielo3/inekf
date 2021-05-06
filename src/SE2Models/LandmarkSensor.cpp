#include "SE2Models/LandmarkSensor.h"

namespace InEKF {

LandmarkSensor::LandmarkSensor(double std) {
    error_ = ERROR::RIGHT;
    M_ = Eigen::Matrix2d::Identity() *std*std;
}

void LandmarkSensor::sawLandmark(int idx, const SE2<Eigen::Dynamic>& state){
    b_ = Eigen::VectorXd::Zero(state().cols());
    b_(2) = 1;
    b_(idx) = -1;

    int rSize = 2;
    int rDim  = 1;

    int curr_cols = state().cols() - rSize;
    int curr_dim  = rDim + rSize*curr_cols; 
    this->H_ = MatrixH::Zero(rSize, curr_dim);
    for(int i=0; i<curr_cols; i++){
        this->H_.block(0, rSize*i+rDim, rSize, rSize) = -1*b_(i+rSize)*MatrixS::Identity();
    }
}

LandmarkSensor::VectorB LandmarkSensor::processZ(const Eigen::VectorXd& z, const SE2<Eigen::Dynamic>& state){
    // convert range and bearing into x and y
    if(z.rows() == 2){
        double r = z(0);
        double b = z(1);
        VectorB z_ = b_;
        z_(0) = r*sin(b);
        z_(1) = r*cos(b);

        return z_;
    }
    else{
        return z;
    }
}

}