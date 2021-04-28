#ifndef PYTHON_MEASUREMODEL
#define PYTHON_MEASUREMODEL

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>

#include <string>

#include <Eigen/Core>
#include <InEKF/Core>

namespace py = pybind11;
using namespace pybind11::literals;

template <class Group>
class PyMeasureModel : public InEKF::MeasureModel<Group> {
    public:
        // Use constructors
        using InEKF::MeasureModel<Group>::MeasureModel;
        typedef typename InEKF::MeasureModel<Group>::MatrixS MatrixS;
        typedef typename InEKF::MeasureModel<Group>::MatrixH MatrixH;
        typedef typename InEKF::MeasureModel<Group>::VectorV VectorV;
        typedef typename InEKF::MeasureModel<Group>::VectorB VectorB;

        VectorB processZ(const Eigen::VectorXd& z, const Group& state) override {
            PYBIND11_OVERRIDE(
                VectorB,                    /* Return type */
                InEKF::MeasureModel<Group>, /* Parent class */
                processZ,                   /* Name of function in C++ (must match Python name) */
                z, state                    /* Argument(s) */
            );
        }

        VectorV calcV(const VectorB& z, const Group& state) override {
            PYBIND11_OVERRIDE(
                VectorV,     
                InEKF::MeasureModel<Group>,
                calcV,
                z, state
            );
        }

        MatrixS calcSInverse(const Group& state) override {
            PYBIND11_OVERRIDE(
                MatrixS,     
                InEKF::MeasureModel<Group>,
                calcSInverse,
                state
            );
        }

        // Make a few protected things available
        using InEKF::MeasureModel<Group>::M_;
        using InEKF::MeasureModel<Group>::error_;

};

template<class T>
void makeMeasure(py::module &m, std::string name){
    using K = InEKF::MeasureModel<T>;

    // For use in defining constructors
    typedef typename K::MatrixH MatrixH;
    typedef typename K::MatrixS MatrixS;
    typedef typename K::VectorB VectorB;

    std::string nameMM = "MeasureModel_" + name;
    py::class_<K, PyMeasureModel<T>> myClass(m, nameMM.c_str());
    myClass
        .def(py::init<>())
        .def(py::init<MatrixH, MatrixS, InEKF::ERROR>())
        .def(py::init<MatrixS, InEKF::ERROR>())

        // Overrideable methods
        .def("processZ", &K::processZ,
            "z"_a, "state"_a)
        .def("calcV", &K::calcV,
            "z"_a, "state"_a)
        .def("calcSInverse", &K::calcSInverse,
            "state"_a)

        // Properties
        .def_property("H", &K::getH, &K::setH)
        .def_readwrite("M", &PyMeasureModel<T>::M_)
        .def_readwrite("error", &PyMeasureModel<T>::error_);
}

template <class T>
void makeGenericMeasure(py::module &m, std::string name){
    using G = InEKF::GenericMeasureModel<T>;

    // For use in defining constructors
    typedef typename G::MatrixH MatrixH;
    typedef typename G::MatrixS MatrixS;
    typedef typename G::VectorB VectorB;

    std::string nameGM = "GenericMeasureModel_" + name;
    py::class_<G, InEKF::MeasureModel<T>> myGenericClass(m, nameGM.c_str());
    myGenericClass
        .def(py::init<VectorB, MatrixS, InEKF::ERROR>())
        .def("processZ", &G::processZ,
            "z"_a, "state"_a);
}

#endif // PYTHON_MEASUREMODEL