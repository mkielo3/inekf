#ifndef STATE
#define STATE

#include <Eigen/Dense>

class State {

    public:
        State();
        State(Eigen::MatrixXd Mu, Eigen::MatrixXd Sigma) : Mu_(Mu), Sigma_(Sigma) {};
        const Eigen::Matrix3d getRotation();
        const Eigen::Vector3d getVelocity();
        const Eigen::Vector3d getPosition();
        const Eigen::MatrixXd getMu();
        const Eigen::MatrixXd getSigma();
        const Eigen::VectorXd getLastu();

        void setRotation(Eigen::Matrix3d R);
        void setVelocity(Eigen::Vector3d v);
        void setPosition(Eigen::Vector3d p);
        void setSigma(Eigen::MatrixXd Sigma);
        void setLastu(Eigen::VectorXd u);

    private:
        Eigen::MatrixXd Mu_;
        Eigen::MatrixXd Sigma_;
        Eigen::MatrixXd Bias_;
        Eigen::VectorXd Last_u_;
};

#endif // STATE