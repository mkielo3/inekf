#ifndef BASE_LIE
#define BASE_LIE

#include <Eigen/Dense>
#include <string>
#include <map>

class LieBase {
    
    public:
        LieBase();
        virtual Eigen::MatrixXd Adjoint(const Eigen::MatrixXd& x);
        virtual Eigen::MatrixXd Mountain(const Eigen::VectorXd& xi);
        virtual Eigen::MatrixXd Cross(const Eigen::VectorXd& xi);
        virtual Eigen::MatrixXd ExpMountain(const Eigen::Vector3d& xi);
};

#endif // BASE_LIE