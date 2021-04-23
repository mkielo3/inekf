// #ifndef PYTHON_SO
// #define PYTHON_SO

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>

#include <string>

#include <Eigen/Core>
#include <InEKF/Core>

namespace py = pybind11;
using namespace pybind11::literals;

#include "LieGroup.cpp"

/*********************** GLOBAL VARIABLES ***********************/
/********** USE TO DEFINE HOW MANY TEMPLATES ARE MADE **********/
constexpr int SO_AUG = 5;


template <int A>
void forLoopSO(py::module &m){
    // make pyclasses
    py::class_<InEKF::SO2<A>> mySO2 = make_group<InEKF::SO2<A>>(m, "SO2", A);
    py::class_<InEKF::SO3<A>> mySO3 = make_group<InEKF::SO3<A>>(m, "SO3", A);

    // Fill in SO2 Constructors
    int a = A == Eigen::Dynamic ? 0 : A;
    int c = A == Eigen::Dynamic ? 1 : A+1;
    typedef typename InEKF::SO2<A>::TangentVector SO2_TV;
    typedef typename InEKF::SO2<A>::MatrixCov SO2_MC;
    typedef typename InEKF::SO2<A>::MatrixState SO2_MS;
    typedef typename InEKF::SO2<A>::VectorAug SO2_VA;
    mySO2.def(py::init<SO2_MS, SO2_MC, SO2_VA>(), 
            "State"_a=SO2_MS::Identity(), "Cov"_a=SO2_MC::Zero(c,c), "Aug"_a=SO2_VA::Zero(a))
        .def(py::init<InEKF::SO2<A> const &>())
        .def(py::init<SO2_TV, SO2_MC>(),
            "xi"_a, "Cov"_a=SO2_MC::Zero(c,c))
        .def(py::init<double, SO2_MC, SO2_VA>(),
            "theta"_a, "Cov"_a=SO2_MC::Zero(c,c), "Aug"_a=SO2_VA::Zero(a));

    // Fill in SO3 Constructors
    c = A == Eigen::Dynamic ? 3 : A+3;
    typedef typename InEKF::SO3<A>::TangentVector SO3_TV;
    typedef typename InEKF::SO3<A>::MatrixCov SO3_MC;
    typedef typename InEKF::SO3<A>::MatrixState SO3_MS;
    typedef typename InEKF::SO3<A>::VectorAug SO3_VA;
    mySO3.def(py::init<SO3_MS, SO3_MC, SO3_VA>(), 
            "State"_a=SO3_MS::Identity(), "Cov"_a=SO3_MC::Zero(c,c), "Aug"_a=SO3_VA::Zero(a))
        .def(py::init<InEKF::SO3<A> const &>())
        .def(py::init<SO3_TV, SO3_MC>(),
            "xi"_a, "Cov"_a=SO3_MC::Zero(c,c))
        .def(py::init<double, double, double, SO3_MC, SO3_VA>(),
            "w1"_a, "w2"_a, "w3"_a, "Cov"_a=SO3_MC::Zero(c,c), "Aug"_a=SO3_VA::Zero(a));

    // iterate
    forLoopSO<A-1>(m);
}

template <>
void forLoopSO<-2>(py::module &m){}

void makeSO(py::module &m) { forLoopSO<SO_AUG>(m); }