#ifndef BASE_PROCESS
#define BASE_PROCESS

#include <Eigen/Dense>
#include "Core/LieGroup.h"

namespace InEKF {

template <class Class, class Group, class U>
class ProcessModel {

    public:
        typedef typename Group::MatrixCov MatrixCov;
        typedef Group myGroup;
        typedef U myU;

    protected:
        MatrixCov Q_;

    public:
        ProcessModel() {};
        static Group f(const U& u, double dt, const Group& state) {
            return Class::f(u, dt, state);
        }
        static MatrixCov MakePhi(const U& u, double dt, const Group& state, ERROR error){
            return Class::MakePhi(u, dt, state, error);
        }

        MatrixCov getQ() const { return Q_; };
        void setQ(MatrixCov Q) { Q_ = Q; };



};

}

#endif // BASE_PROCESS