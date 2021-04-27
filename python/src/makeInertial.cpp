#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>

#include <string>

#include <Eigen/Core>

#include <InEKF/Core>
#include <InEKF/Inertial>

namespace py = pybind11;
using namespace pybind11::literals;

void makeInertial(py::module &m){
    using BaseProcess = InEKF::ProcessModel<InEKF::SE3<2,6>, Eigen::Vector<double,6>>;
    using BaseMeasure = InEKF::MeasureModel<InEKF::SE3<2,6>>;
    // NOTE: Don't have to put in overriden functions if include base classes
    // Just have to put in new classes

    // InertialProcess
    py::class_<InEKF::InertialProcess, BaseProcess, std::shared_ptr<InEKF::InertialProcess>>(m, "InertialProcess")
        .def(py::init<>())
        .def("setGyroNoise", &InEKF::InertialProcess::setGyroNoise,
            "std"_a)
        .def("setAccelNoise", &InEKF::InertialProcess::setAccelNoise,
            "std"_a)
        .def("setGyroBiasNoise", &InEKF::InertialProcess::setGyroBiasNoise,
            "std"_a)
        .def("setAccelBiasNoise", &InEKF::InertialProcess::setAccelBiasNoise,
            "std"_a);

    // DVLSensor
    py::class_<InEKF::DVLSensor, BaseMeasure>(m, "DVLSensor")
        .def(py::init<>())
        .def(py::init<Eigen::Matrix3d, Eigen::Vector3d>(),
            "dvlR"_a, "dvlT"_a)
        .def(py::init<InEKF::SE3<>>(),
            "dvlH"_a)
        .def(py::init<InEKF::SO3<>, Eigen::Vector3d>(),
            "dvlH"_a, "dvlT"_a)
        
        .def("setNoise", &InEKF::DVLSensor::setNoise,
            "std_dvl"_a, "std_imu"_a);

    // DepthSensor
    py::class_<InEKF::DepthSensor, BaseMeasure>(m, "DepthSensor")
        .def(py::init<>())
        .def(py::init<double>(),
            "std"_a)

        .def("setNoise", &InEKF::DepthSensor::setNoise);
}