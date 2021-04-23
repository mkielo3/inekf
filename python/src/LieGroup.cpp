#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>

#include <string>

#include <Eigen/Core>
#include <InEKF/Core>

namespace py = pybind11;
using namespace pybind11::literals;

template <class T>
py::class_<T> make_group(py::module &m, std::string name, int num1, int num2=-10){
    // parse the name
    name += "_";
    name += num1 == Eigen::Dynamic ? "D" : std::to_string(num1);
    if(num2 != -10){
        name += "_"; 
        name += num2 == Eigen::Dynamic ? "D" : std::to_string(num2);
    }
    
    // TODO: Need dummy a,c to help with default dynamic types?
    py::class_<T> myClass = py::class_<T>(m, name.c_str())
        // Getters
        .def("R", &T::R)
        .def_property_readonly("isUncertain", &T::Uncertain)
        .def_property("State", &T::operator(), &T::setState)
        .def_property("Cov", &T::Cov, &T::setCov)
        .def_property("Aug", &T::Aug, &T::setAug)

        // self operators
        .def("inverse", &T::inverse)
        .def("Ad", py::overload_cast<>(&T::Ad, py::const_))
        .def("log", &T::log)

        // Group action
        .def(py::self * py::self)

        // Static Operators
        .def_static("Wedge", &T::Wedge, "xi"_a)
        .def_static("Exp", &T::Exp, "xi"_a)
        .def_static("Log", &T::Exp, "g"_a)
        .def_static("Adjoint", py::overload_cast<const T&>(&T::Ad), "g"_a)

        // Misc
        .def("addAug", &T::addAug, "x"_a, "sigma"_a=1)
        .def("__str__", &T::toString);

    return myClass;
}