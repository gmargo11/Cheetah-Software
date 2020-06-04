#include <pybind11/pybind11.h>
namespace py = pybind11;

#include "Collision/CollisionPlane.h"
//#include "DrawList.h"
#include "Dynamics/Cheetah3.h"
#include "Dynamics/DynamicsSimulator.h"
#include "Dynamics/FloatingBaseModel.h"
#include "Dynamics/MiniCheetah.h"
#include "Dynamics/Quadruped.h"
#include "Dynamics/SpatialInertia.h"
//#include "Graphics3D.h"
//#include "SimControlPanel.h"
//#include "SimulationRunnerHeadless.h"
//#include "Simulation.h"
#include "Utilities/utilities.h"
#include "Utilities/SegfaultHandler.h"

PYBIND11_MODULE(pycheetah, m) {

	py::class_<FloatingBaseModel<float>>(m, "FloatingBaseModel")
		.def(py::init<>())
		.def("addBase", py::overload_cast<float,  const Vec3<float>&, const Mat3<float>&>(&FloatingBaseModel<float>::addBase), "", py::arg("mass"), py::arg("com"), py::arg("I"));
		//.def("addBase", &FloatingBaseModel<float>::addBase(mass, const Vec3<T>& com, const Mat3<T>& I));//float, Vec3<float>&, Mat3<float>&));
		//.def("addBody", &FloatingBaseModel<float>::addBody)
		//.def("addGroundContactPoint", &FloatingBaseModel<float>::addGroundContactPoint);
		//.def("setState", &FloatingBaseModel::addBase)
        
}
