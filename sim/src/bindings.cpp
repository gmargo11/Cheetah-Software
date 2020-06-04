#include <pybind11/pybind11.h>
namespace py = pybind11;

#include "Collision/CollisionPlane.h"
#include "DrawList.h"
#include "Dynamics/Cheetah3.h"
#include "Dynamics/DynamicsSimulator.h"
#include "Dynamics/FloatingBaseModel.h"
#include "Dynamics/MiniCheetah.h"
#include "Dynamics/Quadruped.h"
#include "Graphics3D.h"
#include "SimControlPanel.h"
#include "SimulationRunnerHeadless.h"
#include "Simulation.h"
#include "Utilities/utilities.h"
#include "Utilities/SegfaultHandler.h"

PYBIND11_MODULE(cheetahctrl, m) {

    py::class_<FloatingBaseModel>(m, "FloatingBaseModel")
        .def(py::init<>())
        .def("addBase", &FloatingBaseModel::addBase)
        .def("addBody", &FloatingBaseModel::addBody)
        .def("addGroundContactPoint", &FloatingBaseModel::addGroundContactPoint)
        //.def("setState", &FloatingBaseModel::addBase)
        
}

