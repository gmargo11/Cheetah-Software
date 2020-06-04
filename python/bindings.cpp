#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
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

#include "WBC_Ctrl/WBC_Ctrl.hpp"
#include "WBC_Ctrl/LocomotionCtrl/LocomotionCtrl.hpp"
#include "FSM_States/ControlFSMData.h"
#include "MIT_Controller.hpp"
#include "robot/include/RobotRunner.h"
#include "Utilities/PeriodicTask.h"

PYBIND11_MODULE(pycheetah, m) {

	py::class_<FloatingBaseModel<float>>(m, "FloatingBaseModel")
		.def(py::init<>())
		.def("addBase", py::overload_cast<float,  const Vec3<float>&, const Mat3<float>&>(&FloatingBaseModel<float>::addBase), "", py::arg("mass"), py::arg("com"), py::arg("I"))
	        .def("addGroundContactPoint", py::overload_cast<int, const Vec3<float>&, bool>(&FloatingBaseModel<float>::addGroundContactPoint), "", py::arg("bodyID"), py::arg("location"), py::arg("isFoot"));
		//.def("addBase", &FloatingBaseModel<float>::addBase(mass, const Vec3<T>& com, const Mat3<T>& I));//float, Vec3<float>&, Mat3<float>&));
		//.def("addBody", &FloatingBaseModel<float>::addBody)
		//.def("addGroundContactPoint", &FloatingBaseModel<float>::addGroundContactPoint);
		//.def("setState", &FloatingBaseModel::addBase)
	
        py::class_<LocomotionCtrlData<float>>(m, "LocomotionCtrlData")
		.def(py::init<>())
	        .def_readwrite("pBody_des", &LocomotionCtrlData<float>::pBody_des)
		.def_readwrite("vBody_des", &LocomotionCtrlData<float>::vBody_des)
		.def_readwrite("aBody_des", &LocomotionCtrlData<float>::aBody_des)
		.def_readwrite("pBody_RPY_des", &LocomotionCtrlData<float>::pBody_RPY_des)
		.def_readwrite("vBody_Ori_des", &LocomotionCtrlData<float>::vBody_Ori_des)
		.def_readonly("pFoot_des", &LocomotionCtrlData<float>::pFoot_des);
		//.def_readwrite("vFoot_des", &LocomotionCtrlData<float>::vFoot_des)
		//.def_readwrite("aFoot_des", &LocomotionCtrlData<float>::aFoot_des)
		//.def_readwrite("Fr_des", &LocomotionCtrlData<float>::Fr_des)
		//.def_readwrite("contact_state", &LocomotionCtrlData<float>::contact_state);

	py::class_<LocomotionCtrl<float>> locomotionctrl(m, "LocomotionCtrl");
	locomotionctrl.def(py::init<FloatingBaseModel<float>>());
	locomotionctrl.def("run", py::overload_cast<void *, ControlFSMData<float> &>(&LocomotionCtrl<float>::run), "", py::arg("input"), py::arg("data"));

        py::class_<ControlFSMData<float>> fsmdata(m, "ControlFSMData");
	fsmdata.def(py::init<>(), "Initialize the Control FSM Data");

        py::class_<RobotController> robotctrl(m, "RobotController");

	py::class_<MIT_Controller> mitctrl(m, "MIT_Controller", robotctrl);
	mitctrl.def(py::init<>(), "");
	mitctrl.def("initializeController", py::overload_cast<>(&MIT_Controller::initializeController), "");
	mitctrl.def("runController", py::overload_cast<>(&MIT_Controller::runController), "");
	mitctrl.def("getUserControlParameters", py::overload_cast<>(&MIT_Controller::getUserControlParameters), "");

	py::class_<PeriodicTaskManager>(m, "PeriodicTaskManager")
		.def(py::init<>());

        py::class_<RobotRunner> robotrunner(m, "RobotRunner");
	robotrunner.def(py::init<RobotController*, PeriodicTaskManager*, float, std::string>());
        robotrunner.def("init", py::overload_cast<>(&RobotRunner::init), "");	
        robotrunner.def("run", py::overload_cast<>(&RobotRunner::run), "");

}
