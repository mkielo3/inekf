#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>

#include <string>

#include <Eigen/Core>
#include <InEKF/Core>

#include "LieGroup.cpp"

namespace py = pybind11;
using namespace pybind11::literals;

/*********************** GLOBAL VARIABLES ***********************/
/********** USE TO DEFINE HOW MANY TEMPLATES ARE MADE **********/
constexpr int SE_AUG = 6;
constexpr int SE_COL = 3;


template <int C, int A>
void forLoopSE(py::module &m){
    // make pyclasses
    py::class_<InEKF::SE2<C,A>> mySE2 = make_group<InEKF::SE2<C,A>>(m, "SE2", C, A);
    py::class_<InEKF::SE3<C,A>> mySE3 = make_group<InEKF::SE3<C,A>>(m, "SE3", C, A);

    // Fill in SE2 Constructors
    int a = A == Eigen::Dynamic ? 0 : A;
    int c = (A == Eigen::Dynamic || C == Eigen::Dynamic) ? 3 : InEKF::calcStateDim(2,C,A);
    int ma = C == Eigen::Dynamic ? 3 : InEKF::calcStateMtxSize(2,C);
    typedef typename InEKF::SE2<C,A>::TangentVector SE2_TV;
    typedef typename InEKF::SE2<C,A>::MatrixCov SE2_MC;
    typedef typename InEKF::SE2<C,A>::MatrixState SE2_MS;
    typedef typename InEKF::SE2<C,A>::VectorAug SE2_VA;
    mySE2.def(py::init<SE2_MS, SE2_MC, SE2_VA>(), 
                "State"_a=SE2_MS::Identity(ma,ma), "Cov"_a=SE2_MC::Zero(c,c), "Aug"_a=SE2_VA::Zero(a))
        .def(py::init<InEKF::SE2<C,A> const &>())
        .def(py::init<SE2_TV, SE2_MC>(),
            "xi"_a, "Cov"_a=SE2_MC::Zero(c,c));
        // .def(py::init<double, double, double, SE2_MC, SE2_VA>(),
            // "theta"_a, "x"_a, "y"_a, "Cov"_a=SE2_MC::Zero(c,c), "Aug"_a=SE2_VA::Zero(a));

    // Fill in SE3 Constructors
    c = (A == Eigen::Dynamic || C == Eigen::Dynamic) ? 6 : InEKF::calcStateDim(3,C,A);
    ma = C == Eigen::Dynamic ? 4 : InEKF::calcStateMtxSize(3,C);
    typedef typename InEKF::SE3<C,A>::TangentVector SE3_TV;
    typedef typename InEKF::SE3<C,A>::MatrixCov SE3_MC;
    typedef typename InEKF::SE3<C,A>::MatrixState SE3_MS;
    typedef typename InEKF::SE3<C,A>::VectorAug SE3_VA;
    mySE3.def(py::init<SE3_MS, SE3_MC, SE3_VA>(), 
                "State"_a=SE3_MS::Identity(ma,ma), "Cov"_a=SE3_MC::Zero(c,c), "Aug"_a=SE3_VA::Zero(a))
        .def(py::init<InEKF::SE3<C,A> const &>())
        .def(py::init<SE3_TV, SE3_MC>(),
            "xi"_a, "Cov"_a=SE3_MC::Zero(c,c));
        // .def(py::init<double, double, double, double, double, double, SE3_MC, SE3_VA>(),
        //     "w1"_a, "w2"_a, "w3"_a, "x"_a, "y"_a, "z"_a, "Cov"_a=SE3_MC::Zero(c,c), "Aug"_a=SE3_VA::Zero(a))
        // .def(py::init<InEKF::SO3<>, Eigen::Matrix<double,InEKF::calcStateDim(3,C,A)-3,1>, SE3_MC>,
                // "R"_a, "xi"_a, "Cov"_a=SE3_MC::Zero(c,c));
    // TODO AddCol
    // TODO Fix these constructors freaking out!

    // iterate
    // TODO Figure out double for loop!
    forLoopSE<A-1, A-1>(m);
}

template <>
void forLoopSE<1,1>(py::module &m){}

void makeSE(py::module &m){ forLoopSE<4,4>(m); }