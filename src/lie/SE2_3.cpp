#include "lie/SE2_3.h"

Eigen::MatrixXd SE2_3::Adjoint(const Eigen::MatrixXd& X){
    Eigen::Matrix3d R = X.block<3,3>(0,0);
    Eigen::Matrix3d v_cross = Cross( X.block<3,1>(0,3) );
    Eigen::Matrix3d p_cross = Cross( X.block<3,1>(0,4) );

    Eigen::MatrixXd Adj = Eigen::MatrixXd::Zero(9, 9);
    for(int i=0; i<3; i++){
        Adj.block<3,3>(3*i, 3*i) = R;
    }
    Adj.block<3,3>(3,0) = v_cross*R;
    Adj.block<3,3>(6,0) = p_cross*R;

    return Adj;
}

Eigen::MatrixXd SE2_3::Mountain(const Eigen::VectorXd& xi){
    Eigen::MatrixXd X = Eigen::MatrixXd::Zero(5,5);
    X.block<3,3>(0,0) = Cross( xi.head(3) );
    X.block<3,1>(0,3) = xi.segment<3>(3);
    X.block<3,1>(0,4) = xi.tail(3);

    return X;
}

Eigen::MatrixXd SE2_3::Cross(const Eigen::VectorXd& xi){
    Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
    M << 0, -xi[2], xi[1],
         xi[2], 0, -xi[0], 
        -xi[1], xi[0], 0;
    return M;
}

Eigen::MatrixXd SE2_3::ExpMountain(const Eigen::VectorXd& xi){
    return Mountain(xi).exp();
}
