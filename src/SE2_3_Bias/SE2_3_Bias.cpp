#include "SE2_3_Bias/SE2_3_Bias.h"

namespace InEKF {

Eigen::MatrixXd SE2_3_Bias::Adjoint(const Eigen::MatrixXd& X){
    Eigen::Matrix3d R = X.block<3,3>(0,0);
    Eigen::Matrix3d v_cross = Cross( X.block<3,1>(0,3) );
    Eigen::Matrix3d p_cross = Cross( X.block<3,1>(0,4) );

    Eigen::MatrixXd Adj = Eigen::MatrixXd::Zero(15, 15);
    for(int i=0; i<3; i++){
        Adj.block<3,3>(3*i, 3*i) = R;
    }
    for(int i=3; i<5; i++){
        Adj.block<3,3>(3*i, 3*i) = Eigen::MatrixXd::Identity(3,3);
    }
    Adj.block<3,3>(3,0) = v_cross*R;
    Adj.block<3,3>(6,0) = p_cross*R;

    return Adj;
}

Eigen::MatrixXd SE2_3_Bias::Mountain(const Eigen::VectorXd& xi){
    Eigen::MatrixXd X = Eigen::MatrixXd::Zero(5,5);
    X.block<3,3>(0,0) = Cross( xi.head(3) );
    X.block<3,1>(0,3) = xi.segment<3>(3);
    X.block<3,1>(0,4) = xi.tail(3);

    return X;
}

Eigen::MatrixXd SE2_3_Bias::Cross(const Eigen::VectorXd& xi){
    Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
    M << 0, -xi[2], xi[1],
         xi[2], 0, -xi[0], 
        -xi[1], xi[0], 0;
    return M;
}

Eigen::MatrixXd SE2_3_Bias::ExpMountain(const Eigen::VectorXd& xi){
    return Mountain(xi).exp();
}

Eigen::MatrixXd SE2_3_Bias::ExpCross(const Eigen::VectorXd& xi){
    return Cross(xi).exp();
}

}