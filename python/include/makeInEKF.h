#ifndef PYTHON_INEKF
#define PYTHON_INEKF

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>

#include <string>

#include <Eigen/Core>
#include <InEKF/Core>

namespace py = pybind11;
using namespace pybind11::literals;

template<class G, class U>
void makeInEKF(py::module &m, std::string name){
    using P = InEKF::ProcessModel<G,U>;
    using T = InEKF::InEKF<P>;
    typedef Eigen::Matrix<double,G::rotSize,G::N> MatrixH;

    name = "InEKF_" + name;
    py::class_<T> myClass(m, name.c_str(), py::dynamic_attr());
    myClass
        .def(py::init<P*, G, InEKF::ERROR>(),
            "pModel"_a, "state"_a, "error"_a=InEKF::RIGHT)
        
        .def("Predict", &T::Predict,
            "u"_a, "dt"_a=1)
        .def("Update", &T::Update,
            "type"_a, "z"_a)
        .def("_addMeasureModel", &T::addMeasureModel,
            "name"_a, "m"_a)

        .def_property("state", &T::getState, &T::setState);

}

#endif // PYTHON_INEKF