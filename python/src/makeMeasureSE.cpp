#include "makeMeasure.h"
#include "forLoop.h"

void makeAllMeasureSE(py::module &m) { 
    forSE( [&m](auto C, auto A){
        makeMeasure<InEKF::SE2<C,A>>(m, makeNameSE<C,A>("SE2"));
        makeMeasure<InEKF::SE3<C,A>>(m, makeNameSE<C,A>("SE3"));

        makeGenericMeasure<InEKF::SE2<C,A>>(m, makeNameSE<C,A>("SE2"));
        makeGenericMeasure<InEKF::SE3<C,A>>(m, makeNameSE<C,A>("SE3"));
    });
}