#include "LieGroup.h"

namespace py = pybind11;
using namespace pybind11::literals;

/*********************** GLOBAL VARIABLES ***********************/
/********** USE TO DEFINE HOW MANY TEMPLATES ARE MADE **********/
constexpr int SE_AUG = 6;
constexpr int SE_COL = 3;


// Compile time double for loop
template <int C, int A>
struct forLoopSE {
    static void value(py::module &m){
        makeSE2<C,A>(m);
        makeSE3<C,A>(m);
        forLoopSE<C, A-1>::value(m);
    }
};

// Skip 0's for Col
template<int A>
struct forLoopSE<0,A>{
    static void value(py::module &m){
        forLoopSE<-1,A>::value(m);
    }
};

// when we hit -2 for aug, flip back to top
template <int C>
struct forLoopSE<C,-2>{
    static void value(py::module &m){
        forLoopSE<C-1,SE_AUG>::value(m);
    }
};

// ending condition
template <int A>
struct forLoopSE<-2,A> {
    static void value(py::module &m){}
};

void makeSE(py::module &m){ forLoopSE<SE_COL,SE_AUG>::value(m); }