#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>

#include "LieGroup.h"

// Definitions found in SO.cpp and SE.cpp
void makeSO(py::module &m);
void makeSE(py::module &m);

PYBIND11_MODULE(_inekf, m) {
    m.doc() = "Invariant Extended Kalman Filter"; // optional module docstring

    // The amount each of these instantiate is found in
    // the top of SO.cpp and SE.cpp
    makeSO(m);
    makeSE(m);

    // If more LieGroups are needed, can declare them here with appropriate template(s)
    // These template functions found in LieGroup.h
    // makeSO2<11>(m);
    // makeSO3<12>(m);
    // makeSE2<13,14>(m);
    // makeSE3<15,16>(m);
}