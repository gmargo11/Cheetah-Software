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

#include "Controllers/StateEstimatorContainer.h"
#include "Controllers/DummyStateEstimatorContainer.h"

#include "ControlParameters/ControlParameterInterface.h"
#include "SimUtilities/GamepadCommand.h"
#include "SimUtilities/IMUTypes.h"
#include "SimUtilities/SpineBoard.h"
#include "SimUtilities/VisualizationData.h"
#include "SimUtilities/ti_boardcontrol.h"
#include "Utilities/SharedMemory.h"




PYBIND11_MODULE(pycheetah, m) {

	py::class_<FloatingBaseModel<float>>(m, "FloatingBaseModel")
		.def(py::init<>())
		.def("addBase", py::overload_cast<float,  const Vec3<float>&, const Mat3<float>&>(&FloatingBaseModel<float>::addBase), "", py::arg("mass"), py::arg("com"), py::arg("I"))
	        .def("addGroundContactPoint", py::overload_cast<int, const Vec3<float>&, bool>(&FloatingBaseModel<float>::addGroundContactPoint), "", py::arg("bodyID"), py::arg("location"), py::arg("isFoot"));
		//.def("addBase", &FloatingBaseModel<float>::addBase(mass, const Vec3<T>& com, const Mat3<T>& I));//float, Vec3<float>&, Mat3<float>&));
		//.def("addBody", &FloatingBaseModel<float>::addBody)
		//.def("addGroundContactPoint", &FloatingBaseModel<float>::addGroundContactPoint);
		//.def("setState", &FloatingBaseModel::addBase)
       
        py::class_<Quadruped<float>> quadruped(m, "Quadruped");
	quadruped.def("buildModel", py::overload_cast<>(&Quadruped<float>::buildModel), "");


	m.def("buildMiniCheetah", &buildMiniCheetah<float>);//, py::return_value_policy::reference);


        py::class_<LocomotionCtrlData<float>>(m, "LocomotionCtrlData")
		.def(py::init<>())
		.def("setBodyDes", [](LocomotionCtrlData<float> &self, Vec3<float> pBody_des, Vec3<float> vBody_des, Vec3<float> aBody_des, Vec3<float> pBody_RPY_des, Vec3<float> vBody_Ori_des){
			self.pBody_des = pBody_des;
			self.vBody_des = vBody_des;
			self.aBody_des = aBody_des;
			self.pBody_RPY_des = pBody_RPY_des;
			self.vBody_Ori_des = vBody_Ori_des;
			})
		.def("setFootDes", [](LocomotionCtrlData<float> &self, int idx, Vec3<float> pFoot_des, Vec3<float> vFoot_des, Vec3<float> aFoot_des, Vec3<float> Fr_des){
			self.pFoot_des[idx] = pFoot_des;
			self.vFoot_des[idx] = vFoot_des;
			self.aFoot_des[idx] = aFoot_des;
			self.Fr_des[idx] = Fr_des;
			})
		.def("setContactState", [](LocomotionCtrlData<float> &self, Vec4<float> contact_state){
			self.contact_state = contact_state;
			})
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
	locomotionctrl.def("run_stateless", py::overload_cast<void *, ControlFSMData<float> &>(&LocomotionCtrl<float>::run_stateless), "", py::arg("input"), py::arg("data"));

        py::class_<ControlFSMData<float>> fsmdata(m, "ControlFSMData");
	fsmdata.def(py::init<>(), "Initialize the Control FSM Data");

	py::class_<ControlFSM<float>> controlFSM(m, "ControlFSM");
	controlFSM.def(py::init<Quadruped<float>*, StateEstimatorContainer<float>*, LegController<float>*, GaitScheduler<float>*, DesiredStateCommand<float>*, RobotControlParameters*, VisualizationData*, MIT_UserParameters*>());
	controlFSM.def_readonly("data", &ControlFSM<float>::data);
//			self._quadruped = _quadruped; 
//			self._stateEstimator = _stateEstimator;
//			self._legController = _legController;
//			self._desiredStateCommand = _desiredStateCommand;
//			self.controlParameters = controlParameters;
//			self.userParameters = userParameters;
//			self.visualizationData = visualizationData;
//			})


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
        robotrunner.def("setupData", [](RobotRunner &self, Quat<double> orientation, Vec3<double> position,
						Vec3<double> omegaBody, Vec3<double> vBody, Vec3<double> acceleration) {
			//SimulatorToRobotMessage simToRobot;
			std::cout << orientation;
			GamepadCommand gamepadCommand = GamepadCommand();
			SpiData spiData;
			TiBoardData tiBoardData;
			//RobotType robotType;
			VectorNavData vectorNavData;
			CheaterState<double> cheaterState;
			cheaterState.orientation = orientation;
			cheaterState.position = position;
			cheaterState.omegaBody = omegaBody;
			cheaterState.vBody = vBody;
			cheaterState.acceleration = acceleration;
			SpiCommand spiCommand;
			TiBoardCommand tiBoardCommand;
			RobotControlParameters controlParameters;
			VisualizationData visualizationData;
			CheetahVisualization cheetahVisualization;

			std::cout << orientation;

	       	        self.driverCommand = &gamepadCommand;
			self.spiData = &spiData;
	 		self.tiBoardData = &tiBoardData;
			self.robotType = RobotType::MINI_CHEETAH;
			self.vectorNavData = &vectorNavData;
			self.cheaterState = &cheaterState;
			self.spiCommand = &spiCommand;
			self.tiBoardCommand = &tiBoardCommand;
			self.controlParameters = &controlParameters;
			self.visualizationData = &visualizationData;
			self.cheetahMainVisualization = &cheetahVisualization;
			
			self.init();
			});		

	py::class_<StateEstimate<float>> se(m, "StateEstimate");

	
	py::class_<StateEstimatorContainer<float>> secontainer(m, "StateEstimatorContainer");
	//secontainer.def(py::init<CheaterState<double>*, VectorNavData*. LegControllerData<float>*. StateEstimate<float>*, RobotControlParameters*>(), "");
	//secontainer.def_readwrite("_data", _data);	
	
	//py::class_<DummyStateEstimatorContainer<float>> dsecontainer(m, "DummyStateEstimatorContainer");
	//dsecontainer.def(py::init<StateEstimate<float>>(), "");, 

	// Simulator to robot messages
	py::class_<GamepadCommand> gamepadcmd(m, "GamepadCommand");
	gamepadcmd.def(py::init<>(), "");
	
	py::class_<RobotType> robottype(m, "RobotType");

	py::class_<VectorNavData> vnavdata(m, "VectorNavData");
	
	py::class_<CheaterState<double>> cheaterstate(m, "CheaterState");
	
	py::class_<SpiData> spidata(m, "SpiData");
	
	py::class_<TiBoardData>tidata(m, "TiBoardData");
	
	py::class_<ControlParameterRequest> cpreq(m, "ControlParameterRequest");
	
	py::class_<RobotControlParameters> robotctrlparams(m, "RobotControlParameters");
	
	py::class_<VisualizationData> visdata(m, "VisualizationData");
	


}
