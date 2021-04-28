#include "makeMeasure.h"
#include "forLoop.h"

void makeAllMeasureSO(py::module &m) { 
    forSO( [&m](auto A){
        makeMeasure<InEKF::SO2<A>>(m, makeNameSO<A>("SO2"));
        makeMeasure<InEKF::SO3<A>>(m, makeNameSO<A>("SO3"));

        if constexpr(A != 0){
            makeGenericMeasure<InEKF::SO2<A>>(m, makeNameSO<A>("SO2"));
        }
        makeGenericMeasure<InEKF::SO3<A>>(m, makeNameSO<A>("SO3"));
    });
}