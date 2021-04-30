#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>

#include <string>

#include <Eigen/Core>

#include <InEKF/Core>
#include <InEKF/SE2_SLAM>

namespace py = pybind11;
using namespace pybind11::literals;

void makeSE2_SLAM(py::module &m){
    using BaseProcess = InEKF::ProcessModel<InEKF::SE2<>, InEKF::SE2<>>;
    using BaseMeasure = InEKF::MeasureModel<InEKF::SE2<>>;
    // NOTE: Don't have to put in overriden functions if include base classes
    // Just have to put in new classes

    // OdometryProcess
    py::class_<InEKF::OdometryProcess, BaseProcess, std::shared_ptr<InEKF::OdometryProcess>>(m, "OdometryProcess")
        .def(py::init<>())
        .def("setQ", py::overload_cast<double>(&InEKF::OdometryProcess::setQ),
            "q"_a)
        .def("setQ", py::overload_cast<Eigen::Vector3d>(&InEKF::OdometryProcess::setQ),
            "q"_a)
        .def("setQ", py::overload_cast<Eigen::Matrix3d>(&InEKF::OdometryProcess::setQ),
            "q"_a);

    // Landmakr Sensor
    // TODO
}