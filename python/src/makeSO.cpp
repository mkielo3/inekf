#include "LieGroup.h"
#include "globals.h"

template <int A>
void forLoopSO(py::module &m){
    makeSO2<A>(m);
    makeSO3<A>(m);

    // iterate
    forLoopSO<A-1>(m);
}

template <>
void forLoopSO<-2>(py::module &m){}

void makeAllSO(py::module &m) { forLoopSO<AUG>(m); }