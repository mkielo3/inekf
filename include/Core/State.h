#ifndef STATE
#define STATE

#include <Eigen/Dense>
#include "LieGroup.h"

namespace InEKF {

enum class ERROR { LEFT, RIGHT };

class State {

    public:
        State();
        State(LieGroup& lie, ERROR error=ERROR::RIGHT);
        State(int dim, int states, int augment=0, ERROR error=ERROR::RIGHT);
        const Eigen::MatrixXd& getMu() const;
        const Eigen::MatrixXd& getSigma() const;
        const Eigen::VectorXd& getAugment() const;
        const Eigen::VectorXd& getLastu() const;

        void setRotation(const Eigen::Matrix3d& R);
        void setMu(const Eigen::MatrixXd& Mu);
        void setSigma(const Eigen::MatrixXd& Sigma);
        void setAugment(const Eigen::VectorXd& Augment);
        void setLastu(const Eigen::VectorXd& u);

        Eigen::Ref<Eigen::MatrixXd> operator[](int idx);
        Eigen::MatrixXd operator[](int idx) const;

        const ERROR error;

    private:
        Eigen::MatrixXd Mu_;
        Eigen::MatrixXd Sigma_;
        Eigen::VectorXd Augment_;
        Eigen::VectorXd Last_u_;

        int dim;
        int cols;
        int augment;
};

}
#endif // STATE