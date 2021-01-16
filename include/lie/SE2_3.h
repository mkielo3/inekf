#ifndef CLASS_SE2_3
#define CLASS_SE2_3

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include "lie/base.h"

#include <iostream>

class SE2_3: public LieGroup {
    
    public:
        SE2_3() {};
        Eigen::MatrixXd Mountain(const Eigen::VectorXd& xi);
        Eigen::MatrixXd ExpMountain(const Eigen::VectorXd& xi);
        
        Eigen::MatrixXd Cross(const Eigen::VectorXd& xi);
        Eigen::MatrixXd ExpCross(const Eigen::VectorXd& xi);

        Eigen::MatrixXd Adjoint(const Eigen::MatrixXd& X);
};

#endif // CLASS_SE2_3