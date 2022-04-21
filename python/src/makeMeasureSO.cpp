#include "makeMeasure.h"
#include "forLoop.h"

void makeAllMeasureSO(py::module &m) { 
    forSO( [&m](auto A){
        makeMeasure<InEKF::SO2<A>>(m, "SO2"+makeNameSO<A>());
        makeMeasure<InEKF::SO3<A>>(m, "SO3"+makeNameSO<A>());
    });
}