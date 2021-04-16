#ifndef INERTIAL_PROCESS
#define INERTIAL_PROCESS

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include "Core/ProcessModel.h"

namespace InEKF {

class InertialProcess : public ProcessModel<InertialProcess, SO2<>, Eigen::Vector<double,6>> {

    private:
        const Eigen::Vector3d g_;

    public:
        InertialProcess();
        ~InertialProcess(){}
        static Group f(U u, double dt, Group state);
        static MatrixCov MakePhi(const U& u, double dt, const Group& state, ERROR error);
        
        void setGyroNoise(double std);
        void setAccelNoise(double std);
        void setGyroBiasNoise(double std);
        void setAccelBiasNoise(double std);
        

};

}

#endif // INERTIAL_PROCESS