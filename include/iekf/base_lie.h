#ifndef BASE_LIE
#define BASE_LIE

#include <Eigen/Dense>
#include <string>
#include <map>

class LieGroup {
    
    public:
        LieGroup() {};
        virtual Eigen::MatrixXd Mountain(const Eigen::VectorXd& xi) = 0;
        virtual Eigen::MatrixXd ExpMountain(const Eigen::VectorXd& xi) = 0;

        virtual Eigen::MatrixXd Cross(const Eigen::VectorXd& xi) = 0;
        virtual Eigen::MatrixXd ExpCross(const Eigen::VectorXd& xi) = 0;
        
        virtual Eigen::MatrixXd Adjoint(const Eigen::MatrixXd& x) = 0;
};

#endif // BASE_LIE