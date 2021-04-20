#include "Core/SO2.h"
#include "iostream"

namespace InEKF{

// helper functions
template <int aug>
void SO2<aug>::verifySize() {
    // we only care if it's off when it's uncertain && dynamic
    if(isUncertain && aug == Eigen::Dynamic){
        int curr_aug = Aug_.rows();
        int curr_dim = Cov_.rows();
        if(calcStateDim(rotSize, 0, curr_aug) != curr_dim){
            throw std::range_error("Covariance size doesn't match State dimension");
        }
    }
}


template <int aug>
SO2<aug> SO2<aug>::inverse() const{
    MatrixState temp = State_.transpose();
    return SO2(temp);
}

template <int aug>
SO2<aug> SO2<aug>::operator*(const SO2<aug>& rhs) const{
    // Make sure they're both the same size
    if(aug == Eigen::Dynamic && (*this).Aug().rows() != rhs.Aug().rows()){
        throw std::range_error("Dynamic SE2 elements have different Aug");
    }

    // Skirt around composing covariances
    MatrixCov Cov = MatrixCov::Zero(c);
    if(this->Uncertain() && rhs.Uncertain()){
        throw "Can't compose uncertain LieGroups";
    }
    if(this->Uncertain()) Cov = this->Cov();
    if(rhs.Uncertain()) Cov = rhs.Cov();

    // Compose state + augment
    MatrixState State = (*this)() * rhs();
    VectorAug Aug = this->Aug() + rhs.Aug();

    return SO2<aug>(State, Cov, Aug);
}

template <int aug>
typename SO2<aug>::MatrixState SO2<aug>::Wedge(const TangentVector& xi){
    MatrixState State;
    double theta = xi(0);
    State << 0, -theta,
            theta, 0;
    return State;
}

template <int aug>
SO2<aug> SO2<aug>::Exp(const TangentVector& xi){
    double theta = xi(0);
    return SO2(theta,
                MatrixCov::Zero(c,c),
                xi.tail(xi.rows()-1));
}

template <int aug>
typename SO2<aug>::TangentVector SO2<aug>::Log(const SO2& g){
    TangentVector xi(g.Aug().rows()+1);
    xi(0) = atan2(g()(1,0), g()(0,0));
    xi.tail(xi.rows()-1) = g.Aug();
    return xi;
}

}