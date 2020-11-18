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
#include "Controllers/PositionVelocityEstimator.h"
#include "Controllers/OrientationEstimator.h"

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
		.def_readonly("pFoot_des", &LocomotionCtrlData<float>::pFoot_des)
		.def_readonly("vFoot_des", &LocomotionCtrlData<float>::vFoot_des)
		.def_readonly("aFoot_des", &LocomotionCtrlData<float>::aFoot_des)
		.def_readonly("Fr_des", &LocomotionCtrlData<float>::Fr_des)
		.def_readonly("contact_state", &LocomotionCtrlData<float>::contact_state);

	py::class_<LocomotionCtrl<float>> locomotionctrl(m, "LocomotionCtrl");
	locomotionctrl.def(py::init<FloatingBaseModel<float>>());
	//locomotionctrl.def("run", py::overload_cast<void *, ControlFSMData<float> &>(&LocomotionCtrl<float>::run), "", py::arg("input"), py::arg("data"));
	locomotionctrl.def("run", [](LocomotionCtrl<float> &self, LocomotionCtrlData<float> lc_data, ControlFSMData<float> fsm_data){
			//LocomotionCtrlData<float> * _wbc_data;
			//_wbc_data = new LocomotionCtrlData<float>();
			self.run(&lc_data, fsm_data);
			}, "");
	//locomotionctrl.def("run_stateless", py::overload_cast<void *, ControlFSMData<float> &>(&LocomotionCtrl<float>::run_stateless), "", py::arg("input"), py::arg("data"));

        py::class_<ConvexMPCLocomotion> cmpc(m, "ConvexMPCLocomotion");
	cmpc.def(py::init<float, int, MIT_UserParameters*>());
	cmpc.def("initialize", py::overload_cast<>(&ConvexMPCLocomotion::initialize), "");
	cmpc.def("run", py::overload_cast<ControlFSMData<float>&>(&ConvexMPCLocomotion::run<float>), "");
	cmpc.def_readwrite("pBody_des", &ConvexMPCLocomotion::pBody_des);
	cmpc.def_readwrite("vBody_des", &ConvexMPCLocomotion::vBody_des);
	cmpc.def_readwrite("aBody_des", &ConvexMPCLocomotion::aBody_des);
	cmpc.def_readwrite("pBody_RPY_des", &ConvexMPCLocomotion::pBody_RPY_des);
	cmpc.def_readwrite("vBody_Ori_des", &ConvexMPCLocomotion::vBody_Ori_des);
	//cmpc.def_readonly("pFoot_des", &ConvexMPCLocomotion::pFoot_des);
	//cmpc.def_readonly("vFoot_des", &ConvexMPCLocomotion::vFoot_des);
	//cmpc.def_readonly("aFoot_des", &ConvexMPCLocomotion::aFoot_des);
	//cmpc.def_readonly("Fr_des", &ConvexMPCLocomotion::Fr_des);
	cmpc.def("get_pFoot_des", [](ConvexMPCLocomotion &self, int idx){
			return self.pFoot_des[idx];
			});
	cmpc.def("get_vFoot_des", [](ConvexMPCLocomotion &self, int idx){
			return self.vFoot_des[idx];
			});
	cmpc.def("get_aFoot_des", [](ConvexMPCLocomotion &self, int idx){
			return self.aFoot_des[idx];
			});
	cmpc.def("get_Fr_des", [](ConvexMPCLocomotion &self, int idx){
			return self.Fr_des[idx];
			});
	cmpc.def_readwrite("contact_state", &ConvexMPCLocomotion::contact_state);

        py::class_<NeuralMPCLocomotion> nmpc(m, "NeuralMPCLocomotion");
	nmpc.def(py::init<float, int, MIT_UserParameters*>());
	nmpc.def("initialize", py::overload_cast<>(&NeuralMPCLocomotion::initialize), "");
	nmpc.def("run", [](NeuralMPCLocomotion& self, ControlFSMData<float>& data, const Vec3<float>& vel_cmd, const Vec3<float>& vel_rpy_cmd, const DMat<float>& fp_rel, const Vec4<float>& fh_rel_cmd, const Vec4<int>& offsets_cmd, const Vec4<int>& durations_cmd, const float footswing_height, const int iterationsBetweenMPC, const DMat<float> & height_map, const bool use_gait_smoothing, const bool use_vel_smoothing, const bool use_gait_cycling){
	    
			Vec2<float> fp_rel_cmd[4];
			for(int i=0; i<4; i++){
			//    Vec2<float> fp_rel_cmd_i;
			    fp_rel_cmd[i][0] = fp_rel(0 + 2*i);
			    fp_rel_cmd[i][1] = fp_rel(1 + 2*i);
			//    fp_rel_cmd[i] = fp_rel_cmd_i;
			}
			//(void)fp_rel;
			// populate fp_rel_cmd (?)
			self.run(data, vel_cmd, vel_rpy_cmd, fp_rel_cmd, fh_rel_cmd, offsets_cmd, durations_cmd, footswing_height, iterationsBetweenMPC, height_map, use_gait_smoothing, use_vel_smoothing, use_gait_cycling);

	 }, "");

	nmpc.def_readwrite("pBody_des", &NeuralMPCLocomotion::pBody_des);
	nmpc.def_readwrite("vBody_des", &NeuralMPCLocomotion::vBody_des);
	nmpc.def_readwrite("aBody_des", &NeuralMPCLocomotion::aBody_des);
	nmpc.def_readwrite("pBody_RPY_des", &NeuralMPCLocomotion::pBody_RPY_des);
	nmpc.def_readwrite("vBody_Ori_des", &NeuralMPCLocomotion::vBody_Ori_des);
	nmpc.def("get_pFoot_des", [](NeuralMPCLocomotion &self, int idx){
			return self.pFoot_des[idx];
			});
	nmpc.def("get_vFoot_des", [](NeuralMPCLocomotion &self, int idx){
			return self.vFoot_des[idx];
			});
	nmpc.def("get_aFoot_des", [](NeuralMPCLocomotion &self, int idx){
			return self.aFoot_des[idx];
			});
	nmpc.def("get_Fr_des", [](NeuralMPCLocomotion &self, int idx){
			return self.Fr_des[idx];
			});
	nmpc.def_readwrite("contact_state", &NeuralMPCLocomotion::contact_state);
	nmpc.def_readwrite("iterationCounter", &NeuralMPCLocomotion::iterationCounter);
	nmpc.def_readwrite("iterationsBetweenMPC", &NeuralMPCLocomotion::iterationsBetweenMPC);

        py::class_<VisionMPCLocomotion> vmpc(m, "VisionMPCLocomotion");
	vmpc.def(py::init<float, int, MIT_UserParameters*>());
	vmpc.def("initialize", py::overload_cast<>(&VisionMPCLocomotion::initialize), "");
	vmpc.def("run", [](VisionMPCLocomotion& self, ControlFSMData<float>& data, const Vec3<float>& vel_cmd, const DMat<float> & height_map, const DMat<int> & idx_map){
	    
			self.run(data, vel_cmd, height_map, idx_map);

	 }, "");

	vmpc.def_readwrite("pBody_des", &VisionMPCLocomotion::pBody_des);
	vmpc.def_readwrite("vBody_des", &VisionMPCLocomotion::vBody_des);
	vmpc.def_readwrite("aBody_des", &VisionMPCLocomotion::aBody_des);
	vmpc.def_readwrite("pBody_RPY_des", &VisionMPCLocomotion::pBody_RPY_des);
	vmpc.def_readwrite("vBody_Ori_des", &VisionMPCLocomotion::vBody_Ori_des);
	vmpc.def("get_pFoot_des", [](VisionMPCLocomotion &self, int idx){
			return self.pFoot_des[idx];
			});
	vmpc.def("get_vFoot_des", [](VisionMPCLocomotion &self, int idx){
			return self.vFoot_des[idx];
			});
	vmpc.def("get_aFoot_des", [](VisionMPCLocomotion &self, int idx){
			return self.aFoot_des[idx];
			});
	vmpc.def("get_Fr_des", [](VisionMPCLocomotion &self, int idx){
			return self.Fr_des[idx];
			});
	vmpc.def_readwrite("contact_state", &VisionMPCLocomotion::contact_state);
	vmpc.def_readwrite("iterationCounter", &VisionMPCLocomotion::iterationCounter);
	vmpc.def_readwrite("iterationsBetweenMPC", &VisionMPCLocomotion::iterationsBetweenMPC);

	py::class_<ControlFSMData<float>> fsmdata(m, "ControlFSMData");
	fsmdata.def(py::init<>(), "Initialize the Control FSM Data");

	py::class_<ControlFSM<float>> controlFSM(m, "ControlFSM");
	controlFSM.def(py::init<Quadruped<float>*, StateEstimatorContainer<float>*, LegController<float>*, GaitScheduler<float>*, DesiredStateCommand<float>*, RobotControlParameters*, VisualizationData*, MIT_UserParameters*>());
	controlFSM.def_readonly("data", &ControlFSM<float>::data);
        controlFSM.def("initialize", py::overload_cast<>(&ControlFSM<float>::initialize), "");
	controlFSM.def("runFSM", py::overload_cast<>(&ControlFSM<float>::runFSM), "");
	controlFSM.def("printInfo", py::overload_cast<int>(&ControlFSM<float>::printInfo), "");


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
	se.def(py::init<>(), "");
	se.def_readwrite("contactEstimate", &StateEstimate<float>::contactEstimate);
	se.def_readwrite("position", &StateEstimate<float>::position);
	se.def_readwrite("vBody", &StateEstimate<float>::vBody);
	se.def_readwrite("orientation", &StateEstimate<float>::orientation);
	se.def_readwrite("omegaBody", &StateEstimate<float>::omegaBody);
	se.def_readwrite("rBody", &StateEstimate<float>::rBody);
	se.def_readwrite("rpy", &StateEstimate<float>::rpy);
	se.def_readwrite("omegaWorld", &StateEstimate<float>::omegaWorld);
	se.def_readwrite("vWorld", &StateEstimate<float>::vWorld);
	se.def_readwrite("aBody", &StateEstimate<float>::aBody);
	se.def_readwrite("aWorld", &StateEstimate<float>::aWorld);
	
	py::class_<StateEstimatorContainer<float>> secontainer(m, "StateEstimatorContainer");
	secontainer.def(py::init<CheaterState<double>*, VectorNavData*, LegControllerData<float>*, StateEstimate<float>*, RobotControlParameters*>(), "");
	secontainer.def("initializeCheater", [](StateEstimatorContainer<float> &self){
			self.removeAllEstimators();
			self.addEstimator<ContactEstimator<float>>();
			Vec4<float> contactDefault;
			contactDefault << 0.5, 0.5, 0.5, 0.5;
			self.setContactPhase(contactDefault);
			self.addEstimator<CheaterOrientationEstimator<float>>();
			self.addEstimator<CheaterPositionVelocityEstimator<float>>();
			}, "");
	secontainer.def("initializeRegular", [](StateEstimatorContainer<float> &self){
                        self.removeAllEstimators();
			self.addEstimator<ContactEstimator<float>>();
                        Vec4<float> contactDefault;
                        contactDefault << 0.5, 0.5, 0.5, 0.5;
                        self.setContactPhase(contactDefault);
                        self.addEstimator<VectorNavOrientationEstimator<float>>();
                        self.addEstimator<LinearKFPositionVelocityEstimator<float>>();
                        }, "");
	secontainer.def("run", [](StateEstimatorContainer<float> &self){
				self.run(nullptr);
			}, "");
	secontainer.def("getResult", py::overload_cast<>(&StateEstimatorContainer<float>::getResult), "");

	
	// Simulator to robot messages
	py::class_<GamepadCommand> gamepadcmd(m, "GamepadCommand");
	gamepadcmd.def(py::init<>(), "");
	
	py::class_<rc_control_settings> rccmd(m, "rc_control_settings");
	rccmd.def(py::init<>(), "");
	rccmd.def_readwrite("mode", &rc_control_settings::mode);
	rccmd.def_readonly("p_des", &rc_control_settings::p_des);
	rccmd.def_readwrite("height_variation", &rc_control_settings::height_variation);
	rccmd.def_readonly("v_des", &rc_control_settings::v_des);
	rccmd.def("set_v_des", [](rc_control_settings & self, Vec3<float> vel_cmd){
			for(int i=0; i < 3; ++i){
				self.v_des[i] = vel_cmd[i];
			}
		}, "");

	rccmd.def_readonly("rpy_des", &rc_control_settings::rpy_des);
	//rccmd.def_readwrite("omega_des", &rc_control_settings::omega_des);
	//rccmd.def_readwrite("variable", &rc_control_settings::variable);
	
	py::class_<RobotType> robottype(m, "RobotType");
	robottype.def(py::init<>(), "");

	py::class_<VectorNavData> vnavdata(m, "VectorNavData");
	vnavdata.def(py::init<>(), "");
	vnavdata.def_readwrite("accelerometer", &VectorNavData::accelerometer);
	vnavdata.def_readwrite("gyro", &VectorNavData::gyro);
	vnavdata.def_readwrite("quat", &VectorNavData::quat);
	
	py::class_<CheaterState<double>> cheaterstate(m, "CheaterState");
	cheaterstate.def(py::init<>(), "");
	cheaterstate.def_readwrite("orientation", &CheaterState<double>::orientation);
	cheaterstate.def_readwrite("position", &CheaterState<double>::position);
	cheaterstate.def_readwrite("omegaBody", &CheaterState<double>::omegaBody);
	cheaterstate.def_readwrite("vBody", &CheaterState<double>::vBody);
	cheaterstate.def_readwrite("acceleration", &CheaterState<double>::acceleration);
	
	py::class_<SpiData> spidata(m, "SpiData");
	spidata.def(py::init<>(), "");
	spidata.def("setLegData", [](SpiData &self, int idx, float q_abad, float q_hip, float q_knee, float qd_abad, float qd_hip, float qd_knee){
			self.q_abad[idx] = q_abad;
			self.q_hip[idx] = q_hip;
			self.q_knee[idx] = q_knee;
			self.qd_abad[idx] = qd_abad;
			self.qd_hip[idx] = qd_hip;
			self.qd_knee[idx] = qd_knee;
			//self.flag[i] = flag;
			}, "");
	
	py::class_<TiBoardData>tidata(m, "TiBoardData");
	tidata.def(py::init<>(), "");
	
	py::class_<ControlParameterRequest> cpreq(m, "ControlParameterRequest");
	cpreq.def(py::init<>(), "");
	
	py::class_<RobotControlParameters> robotctrlparams(m, "RobotControlParameters");
	robotctrlparams.def(py::init<>(), "");
	robotctrlparams.def("defineAndInitializeFromYamlFile", py::overload_cast<const string& >(&MIT_UserParameters::defineAndInitializeFromYamlFile), "", py::arg("filename"));
	robotctrlparams.def("initializeFromYamlFile", py::overload_cast<const string& >(&MIT_UserParameters::initializeFromYamlFile), "", py::arg("filename"));
	robotctrlparams.def("printToYamlString", [](RobotControlParameters &self) {
			return self.collection.printToYamlString();
			}, "");

	py::class_<MIT_UserParameters> mitctrlparams(m, "MIT_UserParameters");
	mitctrlparams.def(py::init<>(), "");
	mitctrlparams.def("defineAndInitializeFromYamlFile", py::overload_cast<const string& >(&MIT_UserParameters::defineAndInitializeFromYamlFile), "", py::arg("filename"));
	mitctrlparams.def("initializeFromYamlFile", py::overload_cast<const string& >(&MIT_UserParameters::initializeFromYamlFile), "", py::arg("filename"));
	mitctrlparams.def("printToYamlString", [](MIT_UserParameters &self) {
			return self.collection.printToYamlString();
			}, "");
	
	py::class_<VisualizationData> visdata(m, "VisualizationData");
	visdata.def(py::init<>(), "");
	
	py::class_<DesiredStateCommand<float>> desiredstatecmd(m, "DesiredStateCommand");
	desiredstatecmd.def(py::init<GamepadCommand*, rc_control_settings*, RobotControlParameters*, StateEstimate<float>*, float>(), "");
		
	
	py::class_<GaitScheduler<float>> gaitscheduler(m, "GaitScheduler");
	gaitscheduler.def(py::init<MIT_UserParameters*, float>(), "");
	
	py::class_<LegController<float>> legctrl(m, "LegController");
	legctrl.def(py::init<Quadruped<float>&>(), "");
	//legctrl.def("getCommands", [](LegController<float> &self){
	//		return self.commands;
	//		});
	legctrl.def("getCommands", [](LegController<float> &self, int idx){ return self.commands[idx]; }, "");
	legctrl.def_readonly("commands", &LegController<float>::commands);
	legctrl.def("updateData", py::overload_cast<const SpiData*>(&LegController<float>::updateData), "");
	legctrl.def("getData", [](LegController<float> &self, int idx){ return self.datas[idx]; }, "");
	legctrl.def("zeroCommand", py::overload_cast<>(&LegController<float>::zeroCommand), "");
	legctrl.def("setEnabled", py::overload_cast<bool>(&LegController<float>::setEnabled), "");
	
	py::class_<LegControllerCommand<float>> legcmd(m, "LegControllerCommand");
	legcmd.def(py::init<>(), "");
	legcmd.def_readwrite("tauFeedForward", &LegControllerCommand<float>::tauFeedForward);
	legcmd.def_readwrite("forceFeedForward", &LegControllerCommand<float>::forceFeedForward);
	legcmd.def_readwrite("qDes", &LegControllerCommand<float>::qDes);
	legcmd.def_readwrite("qdDes", &LegControllerCommand<float>::qdDes);
	legcmd.def_readwrite("pDes", &LegControllerCommand<float>::pDes);
	legcmd.def_readwrite("vDes", &LegControllerCommand<float>::vDes);
	legcmd.def_readwrite("kpCartesian", &LegControllerCommand<float>::kpCartesian);
	legcmd.def_readwrite("kdCartesian", &LegControllerCommand<float>::kdCartesian);
	legcmd.def_readwrite("kpJoint", &LegControllerCommand<float>::kpJoint);
	legcmd.def_readwrite("kdJoint", &LegControllerCommand<float>::kdJoint);

	
	py::class_<LegControllerData<float>> lcdata(m, "LegControllerData");
	lcdata.def(py::init<>(), "");
	lcdata.def_readwrite("q", &LegControllerData<float>::q);
	lcdata.def_readwrite("qd", &LegControllerData<float>::qd);
	lcdata.def_readwrite("p", &LegControllerData<float>::p);
	lcdata.def_readwrite("v", &LegControllerData<float>::v);
	lcdata.def_readwrite("J", &LegControllerData<float>::J);
	lcdata.def_readwrite("tauEstimate", &LegControllerData<float>::tauEstimate);
	lcdata.def("setQuadruped", py::overload_cast<Quadruped<float> &>(&LegControllerData<float>::setQuadruped), "");


	// Simulation
	//
	/*
	py::class_<DynamicsSimulator<double>> simulator(m, "DynamicsSimulator");
	simulator.def(py::init<FloatingBaseModel<double>& , bool>(), "");
	simulator.def("step", py::overload_cast<double, const DVec<double>&, double, double>(&DynamicsSimulator<double>::step), "");
	simulator.def("setState", py::overload_cast<const FBModelState<double>&>(&DynamicsSimulator<double>::setState), "");
	simulator.def("getState", [](DynamicsSimulator<double> &self){
			return self.getState();
			}, "");
	//simulator.def("getDState", py::overload_cast<>(&DynamicsSimulator<double>::getDState), "");
	simulator.def("addCollisionPlane", py::overload_cast<double, double, double>(&DynamicsSimulator<double>::addCollisionPlane), "");
	simulator.def("getModel", py::overload_cast<>(&DynamicsSimulator<double>::getModel), "");
	simulator.def("getContactForce", py::overload_cast<size_t>(&DynamicsSimulator<double>::getContactForce), "");
	
*/
}
