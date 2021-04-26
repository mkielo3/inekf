#include "ProcessModel.h"
#include "globals.h"

namespace py = pybind11;
using namespace pybind11::literals;

template <int C, int A>
std::string makeNameSE(){
    std::string name = "_";
    name += C == Eigen::Dynamic ? "D" : std::to_string(C);
    name += "_";
    name += A == Eigen::Dynamic ? "D" : std::to_string(A);
    return name;
}

template <int A>
std::string makeNameSO(){
    std::string name = "_";
    name += A == Eigen::Dynamic ? "D" : std::to_string(A);
    return name;
}

// mini for loop for iterating over vectors
template<class G, int V>
struct forLoopVector {
    static void value(py::module &m, std::string name){
        std::string myName = name + "_Vec" + std::to_string(V);
        make_process<G,Eigen::Matrix<double,V,1>>(m,myName);

        forLoopVector<G,V-1>::value(m, name);
    }
};
template<class G>
struct forLoopVector<G,0> {
    static void value(py::module &m, std::string name){}
};

// Compile time double for loop
template <int C, int A>
struct forLoopProcess {
    static void value(py::module &m){
        std::string nameSE2 = "SE2"+makeNameSE<C,A>() + "_SE2"+makeNameSE<C,A>();
        make_process<InEKF::SE2<C,A>,InEKF::SE2<C,A>>(m, nameSE2);
        std::string nameSE3 = "SE3"+makeNameSE<C,A>() + "_SE3"+makeNameSE<C,A>();
        make_process<InEKF::SE3<C,A>,InEKF::SE3<C,A>>(m, nameSE3);

        forLoopVector<InEKF::SE2<C,A>, VEC>::value(m, "SE2"+makeNameSE<C,A>());
        forLoopVector<InEKF::SE3<C,A>, VEC>::value(m, "SE3"+makeNameSE<C,A>());

        forLoopProcess<C, A-1>::value(m);
    }
};

// Skip 0's for Col
template<int A>
struct forLoopProcess<0,A>{
    static void value(py::module &m){
        forLoopProcess<-1,A>::value(m);
    }
};

// when we hit -2 for aug, flip back to top
template <int C>
struct forLoopProcess<C,-2>{
    static void value(py::module &m){
        forLoopProcess<C-1,AUG>::value(m);
    }
};

// Also do SO2/3 when C=-2
template <int A>
struct forLoopProcess<-2,A> {
    static void value(py::module &m){
        std::string nameSO2 = "SO2"+makeNameSO<A>() + "_SO2"+makeNameSO<A>();
        make_process<InEKF::SO2<A>,InEKF::SO2<A>>(m, nameSO2);
        std::string nameSO3 = "SO3"+makeNameSO<A>() + "_SO3"+makeNameSO<A>();
        make_process<InEKF::SO3<A>,InEKF::SO3<A>>(m, nameSO3);

        forLoopVector<InEKF::SO2<A>, VEC>::value(m, "SO2"+makeNameSO<A>());
        forLoopVector<InEKF::SO3<A>, VEC>::value(m, "SO3"+makeNameSO<A>());

        forLoopProcess<-2, A-1>::value(m);
    }
};

// ending condition
template <>
struct forLoopProcess<-2,-2> {
    static void value(py::module &m){}
};


void makeAllProcess(py::module &m) { forLoopProcess<COL,AUG>::value(m); }