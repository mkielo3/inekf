#ifndef ODOMETRY_PROCESS
#define ODOMETRY_PROCESS

#include <Eigen/Dense>

#include "InEKF/Core"

namespace InEKF {

class OdometryProcess : public ProcessModel<OdometryProcess, SE2<>, SE2<>> {

    private:
        const Eigen::Vector3d g_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        OdometryProcess(){}
        ~OdometryProcess(){}
        static SE2<> f(SE2<> u, double dt, SE2<> state);
        static MatrixCov MakePhi(const SE2<>& u, double dt, const SE2<>& state, ERROR error);
        
        void setQ(Eigen::Vector3d q) { this->Q_ = q.asDiagonal(); }
        void setQ(Eigen::Matrix3d q) { this->Q_ = q; }
        void setQ(double q) { this->Q_ = q*Eigen::Matrix3d::Identity(); }
};


}

#endif // ODOMETRY_PROCESS