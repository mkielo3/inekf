#include "LieGroup.h"

/*********************** GLOBAL VARIABLES ***********************/
/********** USE TO DEFINE HOW MANY TEMPLATES ARE MADE **********/
constexpr int SO_AUG = 6;


template <int A>
void forLoopSO(py::module &m){
    makeSO2<A>(m);
    makeSO3<A>(m);

    // iterate
    forLoopSO<A-1>(m);
}

template <>
void forLoopSO<-2>(py::module &m){}

void makeSO(py::module &m) { forLoopSO<SO_AUG>(m); }