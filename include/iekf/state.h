#ifndef STATE
#define STATE

#include <Eigen/Dense>

class State {

    public:
        State();
        State(Eigen::MatrixXd Mu) : Mu_(Mu) {};
        const Eigen::Matrix3d getRotation();
        const Eigen::Vector3d getVelocity();
        const Eigen::Vector3d getPosition();
        void setRotation(Eigen::Matrix3d R);
        void setVelocity(Eigen::Vector3d v);
        void setPosition(Eigen::Vector3d p);

    private:
        Eigen::MatrixXd Mu_;
        Eigen::MatrixXd Sigma_;
        Eigen::MatrixXd Bias_;
};

#endif // STATE