#ifndef BASE_LIE
#define BASE_LIE

#include <Eigen/Dense>

namespace InEKF {

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


template  <class Class, int N>
class LieGroup{

    public:
        LieGroup() {};

        typedef Eigen::Matrix<double, N, 1> TangentVector;

        virtual ~LieGroup() {};

        // helper to automatically cast things
        const Class & derived() const{
            return static_cast<const Class&>(*this);
        }

        // self operators
        Class inverse() const {
            return derived().inverse();
        }
        Class compose(const Class& g) const {
            return derived() * g;
        }
        Eigen::Matrix<double, N, N> Ad(){
            return derived().Ad();
        }

        // static operators
        static Class Exp(const TangentVector& v){
            return Class::Exp(v);
        }
        static TangentVector Log(const Class& g){
            return Class::Log(g);
        }
        static Eigen::Matrix<double, N, N> Ad(const Class& g){
            return Class::Ad(g);
        }

};

}

#endif // BASE_LIE