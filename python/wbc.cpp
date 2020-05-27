#include <pybind11/pybind11.h>
namespace py = pybind11;

#include <Controllers/WBC_Ctrl/LocomotionCtrl/LocomotionCtrl.hpp>


int add(int i, int j) {
    return i + j;
}

PYBIND11_MODULE(example, m) {
    m.doc() = "pybind11 example plugin"; // optional module docstring

    m.def("add", &add, "A function which adds two numbers");
}