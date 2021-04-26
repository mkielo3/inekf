#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>

#include "LieGroup.h"
#include "MeasureModel.h"
#include "ProcessModel.h"

// Definitions found in other cpp files
void makeAllSO(py::module &m);
void makeAllSE(py::module &m);
void makeAllMeasure(py::module &m);
void makeAllProcess(py::module &m);

PYBIND11_MODULE(_inekf, m) {
    m.doc() = "Invariant Extended Kalman Filter"; // optional module docstring

    // The amount each of these instantiate is found in globals.cpp
    // If only need specific templates for a smaller binary, turn these off and select what you need below
    makeAllSO(m);
    makeAllSE(m);

    makeAllMeasure(m);
    makeAllProcess(m);

    py::enum_<InEKF::ERROR>(m, "ERROR")
        .value("RIGHT", InEKF::RIGHT)
        .value("LEFT", InEKF::LEFT);

    // If more LieGroups are needed, can declare them here with appropriate template(s)
    // These template functions found in LieGroup.h
    // makeSO2<11>(m);
    // makeSO3<12>(m);
    // makeSE2<13,14>(m);
    // makeSE3<15,16>(m);
    // make_measure<InEKF::SO2<1>>(m);
    // make_process<InEKF::SO2<1>,Eigen::Vector5d>(m);
}