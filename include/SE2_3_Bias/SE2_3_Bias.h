#ifndef CLASS_SE2_3_Bias
#define CLASS_SE2_3_Bias

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include "Core/LieGroup.h"

namespace InEKF {

class SE2_3_Bias: public LieGroup {
    
    public:
        SE2_3_Bias() { dim=3; cols=2; augment=6; };
        Eigen::MatrixXd Mountain(const Eigen::VectorXd& xi);
        Eigen::MatrixXd ExpMountain(const Eigen::VectorXd& xi);
        
        Eigen::MatrixXd Cross(const Eigen::VectorXd& xi);
        Eigen::MatrixXd ExpCross(const Eigen::VectorXd& xi);

        Eigen::MatrixXd Adjoint(const Eigen::MatrixXd& X);     
};

}

#endif // CLASS_SE2_3_Bias