#include <pybind11/pybind11.h>
namespace py = pybind11;

//#include <user/MIT_Controller/Controllers/WBC_Ctrl/LocomotionCtrl/LocomotionCtrl.hpp>
#include <>
#include "common/include/Dynamics/FloatingBaseModel.h"
//#include "FSM_States/ControlFSMData.h"


PYBIND11_MODULE(cheetahctrl, m) {
    py::class_<LocomotionCtrl>(m, "LocomotionCtrl")
        .def(py::init<FloatingBaseModel>())
        .def("run", &LocomotionCtrl::run)

    py::class_<FloatingBaseModel>(m, "FloatingBaseModel")
        .def(py::init<>())
        .def("addBase", &FloatingBaseModel::addBase)
        .def("addBody", &FloatingBaseModel::addBody)
        .def("addGroundContactPoint", &FloatingBaseModel::addGroundContactPoint)
        //.def("setState", &FloatingBaseModel::addBase)

    
}

