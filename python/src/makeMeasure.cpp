#include "MeasureModel.h"
#include "globals.h"

namespace py = pybind11;
using namespace pybind11::literals;

template <int C, int A>
std::string makeNameSE(std::string name){
    name += "_";
    name += C == Eigen::Dynamic ? "D" : std::to_string(C);
    name += "_";
    name += A == Eigen::Dynamic ? "D" : std::to_string(A);
    return name;
}

template <int A>
std::string makeNameSO(std::string name){
    name += "_";
    name += A == Eigen::Dynamic ? "D" : std::to_string(A);
    return name;
}

// Compile time double for loop
template <int C, int A>
struct forLoopMeasure {
    static void value(py::module &m){
        make_measure<InEKF::SE2<C,A>>(m, makeNameSE<C,A>("SE2"));
        make_measure<InEKF::SE3<C,A>>(m, makeNameSE<C,A>("SE3"));

        makeGenericMeasure<InEKF::SE2<C,A>>(m, makeNameSE<C,A>("SE2"));
        makeGenericMeasure<InEKF::SE3<C,A>>(m, makeNameSE<C,A>("SE3"));

        forLoopMeasure<C, A-1>::value(m);
    }
};

// Skip 0's for Col
template<int A>
struct forLoopMeasure<0,A>{
    static void value(py::module &m){
        forLoopMeasure<-1,A>::value(m);
    }
};

// when we hit -2 for aug, flip back to top
template <int C>
struct forLoopMeasure<C,-2>{
    static void value(py::module &m){
        forLoopMeasure<C-1,AUG>::value(m);
    }
};

// Also do SO2/3 when C=-2
template <int A>
struct forLoopMeasure<-2,A> {
    static void value(py::module &m){
        make_measure<InEKF::SO2<A>>(m, makeNameSO<A>("SO2"));
        make_measure<InEKF::SO3<A>>(m, makeNameSO<A>("SO3"));

        makeGenericMeasure<InEKF::SO2<A>>(m, makeNameSO<A>("SO2"));
        makeGenericMeasure<InEKF::SO3<A>>(m, makeNameSO<A>("SO3"));

        forLoopMeasure<-2, A-1>::value(m);
    }
};

// Skip SO2_0 for Generic Measure
template <>
struct forLoopMeasure<-2,0> {
    static void value(py::module &m){
        make_measure<InEKF::SO2<0>>(m, makeNameSO<0>("SO2"));
        make_measure<InEKF::SO3<0>>(m, makeNameSO<0>("SO3"));

        makeGenericMeasure<InEKF::SO3<0>>(m, makeNameSO<0>("SO3"));

        forLoopMeasure<-2, -1>::value(m);
    }
};

// ending condition
template <>
struct forLoopMeasure<-2,-2> {
    static void value(py::module &m){}
};

void makeAllMeasure(py::module &m) { forLoopMeasure<COL,AUG>::value(m); }