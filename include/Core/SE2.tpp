#include "Core/SE2.h"

namespace InEKF {

// helper functions
template <int cols, int aug>
void SE2<cols,aug>::verifySize() {
    // we only care if it's off when it's uncertain && dynamic
    if(isUncertain && (cols == Eigen::Dynamic || aug == Eigen::Dynamic)){
        int curr_aug = Aug_.rows();
        int curr_cols = State_.rows() - rotSize;
        int curr_dim = Cov_.rows();
        if(calcStateDim(rotSize, curr_cols, curr_aug) != curr_dim){
            throw std::range_error("Covariance size doesn't match State dimension");
        }
    }
}

template <int cols, int aug>
void SE2<cols,aug>::verifyTangentVector(const TangentVector& xi){
    // check to make sure everything is ok if things are dynamic
    if(cols == Eigen::Dynamic && aug == Eigen::Dynamic){
        throw std::range_error("Not supported for double Dynamic type");
    }
    if(cols == Eigen::Dynamic && (xi.rows() - 1 - aug) % 2 != 0){
        throw std::range_error("Tangent Vector size was incorrect");
    }
}

// initialize with theta, x, y
template <>
inline SE2<1,0>::SE2(double theta, double x, double y, const MatrixCov& Cov, const VectorAug& Aug) 
        : Cov_(Cov), Aug_(Aug), isUncertain(Cov != MatrixCov::Zero())  {
    State_ << cos(theta), -sin(theta), x,
                sin(theta),  cos(theta), y,
                0, 0, 1;
}

template <int cols, int aug>
SE2<cols,aug>::SE2(const TangentVector& xi, const MatrixCov& Cov) 
        : Cov_(Cov), isUncertain(!Cov.isZero()) {
    verifyTangentVector(xi);

    // figure out state size for dynamic purposes
    int curr_cols = cols;
    int curr_aug = aug;
    if(cols == Eigen::Dynamic){
        curr_cols = (xi.rows() - 1 - aug) / rotSize;
    }
    else if(aug == Eigen::Dynamic){
        curr_aug  = (xi.rows() - 1 - cols*2);
    }
    int curr_mtxSize = calcStateMtxSize(rotSize, curr_cols);

    // fill it up!
    State_ = MatrixState::Identity(curr_mtxSize, curr_mtxSize);
    State_.block(0,0,2,2) = SO2<>(xi(0))();
    for(int i=0;i<curr_cols;i++){
        State_.block(0,2+i,2,1) = xi.segment(2*i+1,2);
    }
    Aug_ = xi.tail(curr_aug);
    verifySize();
}

template <int cols, int aug>
void SE2<cols,aug>::addCol(const Eigen::Vector2d& x, const Eigen::Matrix2d& sigma){
    if(cols != Eigen::Dynamic) throw std::range_error("Can't add columns, not dynamic");
    
    // Add it into state
    int curr_size = State_.rows();
    MatrixState S = MatrixState::Identity(curr_size+1, curr_size+1);
    S.block(0, 0, curr_size, curr_size) = State_;
    S.block(0,curr_size,2,1) = x;
    State_ = S;

    // Add into Sigma
    if(isUncertain){
        int curr_aug = Aug_.rows();
        int curr_dim = Cov_.rows();
        int mtx_dim = curr_dim - curr_aug;

        MatrixCov C = MatrixCov::Zero(curr_dim+2, curr_dim+2);
        C.topLeftCorner(mtx_dim,mtx_dim) = Cov_.topLeftCorner(mtx_dim,mtx_dim);
        C.bottomRightCorner(curr_aug,curr_aug) = Cov_.bottomRightCorner(curr_aug,curr_aug);
        C.bottomLeftCorner(curr_aug, mtx_dim) = Cov_.bottomLeftCorner(curr_aug, mtx_dim);
        C.topRightCorner(mtx_dim, curr_aug) = Cov_.topRightCorner(mtx_dim, curr_aug);

        C.block(mtx_dim, mtx_dim, 2, 2) = sigma;
        Cov_ = C;
    }
}

template <int cols, int aug>
void SE2<cols,aug>::addAug(double x, double sigma){
    if(aug != Eigen::Dynamic) throw std::range_error("Can't add columns, not dynamic");
    
    // Add it into state
    int curr_size = Aug_.rows();
    VectorAug V(curr_size+1);
    V.head(curr_size) = Aug_;
    V(curr_size) = x;
    Aug_ = V;

    // Add into Sigma
    if(isUncertain){
        int curr_dim = Cov_.rows();

        MatrixCov C = MatrixCov::Zero(curr_dim+1, curr_dim+1);
        C.topLeftCorner(curr_dim,curr_dim) = Cov_;

        C(curr_dim,curr_dim) = sigma;
        Cov_ = C;
    }
}

template <int cols, int aug>
inline SE2<cols,aug> SE2<cols, aug>::Exp(const TangentVector& xi){
    verifyTangentVector(xi);
    double theta = xi(0);

    // Find V
    Eigen::Matrix2d V;
    if(theta < .0001){
        Eigen::Matrix2d thetaWedge = SO2<>::Wedge(xi.segment(0,1));
        V = Eigen::Matrix2d::Identity() + thetaWedge/2 + thetaWedge*thetaWedge/6 + thetaWedge*thetaWedge*thetaWedge/24;
    }
    else{
        V(0,0) = sin(theta);
        V(1,1) = sin(theta);
        V(1,0) = 1 - cos(theta);
        V(0,1) = cos(theta) - 1;
        V /= theta;
    }

    // figure out state size for dynamic purposes
    int curr_cols = cols;
    int curr_aug = aug;
    if(cols == Eigen::Dynamic){
        curr_cols = (xi.rows() - 1 - aug) / rotSize;
    }
    else if(aug == Eigen::Dynamic){
        curr_aug  = (xi.rows() - 1 - cols*2);
    }
    int curr_mtxSize = calcStateMtxSize(rotSize, curr_cols);

    MatrixState X = MatrixState::Identity(curr_mtxSize,curr_mtxSize);
    X.block(0,0,2,2) = SO2<>::Exp(xi.segment(0,1))();
    for(int i=0;i<curr_cols;i++){
        X.block(0,2+i,2,1) = V*xi.segment(2*i+1,2);
    }
    return SE2(X, MatrixCov::Zero(c,c), xi.tail(aug));
}

template <int cols, int aug>
SE2<cols,aug> SE2<cols,aug>::inverse() const{
    int curr_cols = State_.cols() - rotSize;
    int curr_size  = State_.cols();

    MatrixState S = MatrixState::Identity(curr_size,curr_size);
    Eigen::Matrix2d RT = this->R().inverse()();
    S.block(0,0,2,2) = RT;
    for(int i=0;i<curr_cols;i++){
        S.block(0,2+i,2,1) = -1 * RT * (*this)[i];
    }
    return SE2(S);
}

template <int cols, int aug>
SE2<cols,aug> SE2<cols,aug>::operator*(const SE2& rhs) const{
    // Skirt around composing covariances
    MatrixCov Cov = MatrixCov::Zero(c,c);
    if(this->Uncertain() && rhs.Uncertain()){
        throw "Can't compose uncertain LieGroups";
    }
    if(this->Uncertain()) Cov = this->Cov();
    if(rhs.Uncertain()) Cov = rhs.Cov();

    // Compose state + augment
    MatrixState State = (*this)() * rhs();
    VectorAug Aug = this->Aug() + rhs.Aug();

    return SE2(State, Cov, Aug);
}

template <int cols, int aug>
typename SE2<cols,aug>::MatrixState SE2<cols,aug>::Wedge(const TangentVector& xi){
    verifyTangentVector(xi);

    // figure out state size for dynamic purposes
    int curr_cols = cols;
    int curr_aug = aug;
    if(cols == Eigen::Dynamic){
        curr_cols = (xi.rows() - 1 - aug) / rotSize;
    }
    else if(aug == Eigen::Dynamic){
        curr_aug  = (xi.rows() - 1 - cols*2);
    }
    int curr_mtxSize = calcStateMtxSize(rotSize, curr_cols);

    // Fill it in
    MatrixState X = MatrixState::Zero(curr_mtxSize, curr_mtxSize);
    X.block(0,0,2,2) = SO2<>::Wedge(xi.segment(0,1));
    for(int i=0;i<curr_cols;i++){
        X.block(0,2+i,2,1) = xi.segment(2*i+1,2);
    }
    return X;
}

template <int cols, int aug>
typename SE2<cols,aug>::TangentVector SE2<cols,aug>::Log(const SE2& g){
    // figure out state size for dynamic purposes
    int curr_cols = g().cols() - rotSize;
    int curr_aug = g.Aug().rows();
    int curr_dim = calcStateDim(rotSize, curr_cols, curr_aug);

    double theta = SO2<>::Log(g.R())(0);
    double a, b, scale;
    if(theta < .0001){
        a = 1;
        b = 0;
        scale = 1;
    }
    else{
        a = sin(theta)/theta;
        b = (1-cos(theta))/theta;
        scale = 1 / (a*a + b*b);
    }

    Eigen::Matrix2d V_inv;
    V_inv << a, b, -b, a;
    V_inv *= scale;

    TangentVector xi = TangentVector::Zero(curr_dim);
    xi(0) = theta;
    for(int i=0;i<curr_cols;i++){
        xi.segment(1+2*i,2) = V_inv*g[i];
    }
    xi.tail(curr_aug) = g.Aug();
    return xi;
}

template <int cols, int aug>
typename SE2<cols,aug>::MatrixCov SE2<cols,aug>::Ad(const SE2& g){
    // figure out state size for dynamic purposes
    int curr_cols = g().cols() - rotSize;
    int curr_aug = g.Aug().rows();
    int curr_dim = calcStateDim(rotSize, curr_cols, curr_aug);

    MatrixCov Ad_X = MatrixCov::Identity(curr_dim, curr_dim);
    for(int i=0;i<curr_cols;i++){
        Ad_X.block(1+2*i,1+2*i,2,2) = g.R()();
        Ad_X(1+2*i,0) = g[i][1];
        Ad_X(2+2*i,0) = -g[i][0];
    }
    return Ad_X;
}

template <int cols, int aug>
std::ostream& operator<<(std::ostream& os, const SE2<cols, aug>& rhs)  
{
    os << "Matrix\n" << rhs();
    if(rhs.Uncertain()) os << "\nSigma\n" << rhs.Cov();
    if(aug != 0) os << "\nAug\n" << rhs.Aug();
    return os;
}

}