#ifndef ODOMETRY_PROCESS
#define ODOMETRY_PROCESS

#include <Eigen/Core>

#include "InEKF/Core"

namespace InEKF {

class OdometryProcess : public ProcessModel<SE2<>, SE2<>> {

    private:
        const Eigen::Vector3d g_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        OdometryProcess(){}
        OdometryProcess(float theta_std, float x_std, float y_std){
            Eigen::Vector3d q;
            q << theta_std, x_std, y_std;
            setQ(q);
        }
        OdometryProcess(Eigen::Vector3d q){ setQ(q); }
        OdometryProcess(Eigen::Matrix3d q){ setQ(q); }
        ~OdometryProcess(){}
        
        SE2<> f(SE2<> u, double dt, SE2<> state) override;
        MatrixCov MakePhi(const SE2<>& u, double dt, const SE2<>& state, ERROR error) override;
        
        void setQ(Eigen::Vector3d q) { this->Q_ = q.asDiagonal(); }
        void setQ(Eigen::Matrix3d q) { this->Q_ = q; }
        void setQ(double q) { this->Q_ = q*Eigen::Matrix3d::Identity(); }
};


}

#endif // ODOMETRY_PROCESS