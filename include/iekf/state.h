#ifndef STATE
#define STATE

#include <Eigen/Dense>
#include "base_lie.h"

class State {

    public:
        enum ERROR { LEFT, RIGHT };
        State();
        State(LieGroup& lie, ERROR error=State::RIGHT);
        State(int dim, int states, int augment=0, ERROR error=State::RIGHT);
        const Eigen::Matrix3d getRotation();
        const Eigen::MatrixXd getMu();
        const Eigen::MatrixXd getSigma();
        const Eigen::VectorXd getAugment();
        const Eigen::VectorXd getLastu();

        void setRotation(Eigen::Matrix3d R);
        void setMu(Eigen::MatrixXd Mu);
        void setSigma(Eigen::MatrixXd Sigma);
        void setAugment(Eigen::VectorXd Augment);
        void setLastu(Eigen::VectorXd u);

        Eigen::Ref<Eigen::MatrixXd> operator[](int idx);
        Eigen::MatrixXd operator[](int idx) const;

        const ERROR error;

    private:
        Eigen::MatrixXd Mu_;
        Eigen::MatrixXd Sigma_;
        Eigen::MatrixXd Augment_;
        Eigen::VectorXd Last_u_;

        int dim;
        int cols;
        int augment;
};

#endif // STATE