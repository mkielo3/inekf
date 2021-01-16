#ifndef BASE_LIE
#define BASE_LIE

#include <Eigen/Dense>
#include <string>
#include <map>

class LieBase {
    
    public:
        LieBase() {};
        virtual Eigen::MatrixXd Adjoint(const Eigen::MatrixXd& x) = 0;
        virtual Eigen::MatrixXd Mountain(const Eigen::VectorXd& xi) = 0;
        virtual Eigen::MatrixXd Cross(const Eigen::VectorXd& xi) = 0;
        virtual Eigen::MatrixXd ExpMountain(const Eigen::VectorXd& xi) = 0;
};

#endif // BASE_LIE