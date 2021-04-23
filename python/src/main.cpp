#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>

#include <Eigen/Core>
#include <InEKF/Core>

namespace py = pybind11;
using namespace pybind11::literals;


void makeSO(py::module &m);
void makeSE(py::module &m);

PYBIND11_MODULE(_inekf, m) {
    m.doc() = "Invariant Extended Kalman Filter"; // optional module docstring

    makeSE(m);
    makeSO(m);
}