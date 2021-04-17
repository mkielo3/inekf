#ifndef BASE_LIE
#define BASE_LIE

#include <Eigen/Dense>

namespace InEKF {

enum ERROR { LEFT, RIGHT };

constexpr int calcStateDim(int rotMtxSize, int cols, int aug){
    if(rotMtxSize == Eigen::Dynamic || cols == Eigen::Dynamic){
        return Eigen::Dynamic;
    }
    else{
        return rotMtxSize*(rotMtxSize-1)/2 + rotMtxSize*cols + aug;
    }
}
constexpr int calcStateMtxSize(int rotMtxSize, int cols){
    if(rotMtxSize == Eigen::Dynamic || cols == Eigen::Dynamic){
        return Eigen::Dynamic;
    }
    else{
        return rotMtxSize + cols;
    }
}


template  <class Class, int N, int M>
class LieGroup{

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef Eigen::Matrix<double, N, 1> TangentVector;
        typedef Eigen::Matrix<double, N, N> MatrixCov;
        typedef Eigen::Matrix<double, M, M> MatrixState;

        LieGroup() {};

        virtual ~LieGroup() {};

        // helper to automatically cast things
        const Class & derived() const{
            return static_cast<const Class&>(*this);
        }

        // self operations
        Class inverse() const {
            return derived().inverse();
        }
        TangentVector log() const {
            return Class::Log(derived());
        }
        MatrixCov Ad() const{
            return Class::Ad(derived());
        }

        // Group action
        Class compose(const Class& g) const {
            return derived() * g;
        }

        // static operators
        static MatrixState Wedge(const TangentVector& xi){
            return Class::Wedge(xi);
        }
        static Class Exp(const TangentVector& xi){
            return Class::Exp(xi);
        }
        static TangentVector Log(const Class& g){
            return Class::Log(g);
        }
        static MatrixCov Ad(const Class& g){
            return Class::Ad(g);
        }

};

}

#endif // BASE_LIE