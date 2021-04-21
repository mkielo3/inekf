#ifndef BASE_PROCESS
#define BASE_PROCESS

#include <Eigen/Core>
#include "Core/LieGroup.h"

namespace InEKF {

template <class Group, class U>
class ProcessModel {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
        typedef typename Group::MatrixCov MatrixCov;
        typedef typename Group::MatrixState MatrixState;
        typedef Group myGroup;
        typedef U myU;

    protected:
        MatrixCov Q_;

    public:
        ProcessModel() {};
        virtual Group f(U u, double dt, Group state) = 0;
        virtual MatrixCov MakePhi(const U& u, double dt, const Group& state, ERROR error) = 0;

        MatrixCov getQ() const { return Q_; };
        void setQ(MatrixCov Q) { Q_ = Q; };



};

}

#endif // BASE_PROCESS