import numpy as np
import time
import os

import pycheetah
from pycheetah import * #FloatingBaseModel, ControlFSMData, LocomotionCtrl, LocomotionCtrlData, PeriodicTaskManager, RobotRunner, MIT_Controller

import raisimpy as raisim

#fb = FloatingBaseModel()
#fb.addGroundContactPoint(0, np.array([1., 1., 1.]), False)
#fb.addBase(1.0, np.ones(3), np.eye(3))
#print(fb)

class Cheetah:

    def __init__(self, robot_filename, user_filename, dt, model='m'):

        # define structs
        self.robotparams, self.userparams = None, None
        self.cheetah, self.model = None, None
        self.cheaterState, self.vnavData, self.legControllerData, self.stateEstimate, self.stateEstimator = None, None, None, None, None
        self.gamepadCmd, self.rc_command, self.legController, self.gaitScheduler, self.desiredStateCmd = None, None, None, None, None
        self.fsm, self.vizData = None, None

        self.dt = dt

        # load params
        self.load_params(robot_filename, user_filename)

        # make model
        self.make_model(model=model)

        # make state estimator
        self.make_state_estimator()

        # make fsm
        self.make_fsm()

         # set current state
        position = np.array([0.0, 0.0, 0.29])
        orientation = np.array([1.0, 0.0, 0.0, 0.0])
        vBody = np.array([0.0, 0.0, 0.0])
        omegaBody = np.array([0.0, 0.0, 0.0])
        acceleration = np.array([0.0, 0.0, 0.0])

    def load_params(self, robot_filename, user_filename):

        self.robotparams = RobotControlParameters()
        self.userparams = MIT_UserParameters()
        
        # load parameters
        self.robotparams.initializeFromYamlFile(robot_filename)
        self.userparams.initializeFromYamlFile(user_filename)
        #print(userparams.printToYamlString())

    def make_model(self, model):
        # make model
        if model == 'm':
            self.cheetah = buildMiniCheetah()
        else:
            raise ValueError("Only mini cheetah is currently supported by pycheetah!")

        self.model = self.cheetah.buildModel()

    def make_state_estimator(self):
        assert self.cheetah is not None, "self.cheetah not initialized! you should call make_model before make_state_estimator"
        assert self.robotparams is not None, "self.robotparams is not initialized! you should call load_params before make_state_estimator."

        self.cheaterState, self.vnavData, self.legControllerData, self.stateEstimate = CheaterState(), VectorNavData(), LegControllerData(), StateEstimate()
        # cheaterState.orientation, cheaterState.position, cheaterState.omegaBody, cheaterState.vBody, cheaterState.acceleration = orientation, position, omegaBody, vBody, acceleration
        self.stateEstimator = StateEstimatorContainer(self.cheaterState, self.vnavData, self.legControllerData, self.stateEstimate, self.robotparams)
        self.stateEstimator.initializeCheater()

    def make_fsm(self):
        assert self.cheetah is not None, "self.cheetah not initialized! you should call make_model before make_fsm"
        assert self.robotparams is not None, "self.robotparams is not initialized! you should call load_params before make_fsm."
        assert self.userparams is not None, "self.userparams is not initialized! you should call load_params before make_fsm."
        assert self.stateEstimate is not None, "self.stateEstimate is not initialized! you should call make_state_estimator before make_fsm."
        assert self.stateEstimator is not None, "self.stateEstimate is not initialized! you should call make_state_estimator before make_fsm."

        self.gamepadCmd, self.rc_command = GamepadCommand(), rc_control_settings()
        self.desiredStateCmd = DesiredStateCommand(self.gamepadCmd, self.rc_command, self.robotparams, self.stateEstimate, self.dt)
        
        self.gaitScheduler = GaitScheduler(self.userparams, self.dt)
        self.legController = LegController(self.cheetah)

        self.vizData = VisualizationData()
        self.fsm = ControlFSM(self.cheetah, self.stateEstimator, self.legController, self.gaitScheduler, self.desiredStateCmd, self.robotparams, self.vizData, self.userparams)
        self.fsm.initialize()
        print("Run FSM...")
        self.fsm.runFSM()
        #self.fsm.printInfo(1)

    def set_cartesian_state(self, cartesian_state):
        self.cheaterState.orientation = cartesian_state[0:4]
        self.cheaterState.position = cartesian_state[4:7]
        self.cheaterState.omegaBody = cartesian_state[7:10]
        self.cheaterState.vBody = cartesian_state[10:13]
        self.cheaterState.acceleration = cartesian_state[13:16]

        self.stateEstimator.run()

    def set_joint_state(self, joint_state, d_joint_state):
        spiData = SpiData()
        for idx in range(4):
            q_abad, q_hip, q_knee = joint_state[3 * idx], joint_state[3 * idx + 1], joint_state[3 * idx + 2]
            qd_abad, qd_hip, qd_knee = d_joint_state[3 * idx], d_joint_state[3 * idx + 1], d_joint_state[3 * idx + 2]
            spiData.setLegData(idx, q_abad, q_hip, q_knee, qd_abad, qd_hip, qd_knee)
        self.legController.updateData(spiData)
        #print('joint angles: ', self.legController.getData(0).q)
        #print('foot position: ', self.legController.getData(0).p)

    def get_joint_commands(self):
        commands = [self.legController.getCommands(i) for i in range(4)]

        tauff = np.concatenate(([cmd.tauFeedForward for cmd in commands]))
        forceff = np.concatenate(([cmd.forceFeedForward for cmd in commands]))
        qDes = np.concatenate(([cmd.qDes for cmd in commands]))
        qdDes = np.concatenate(([cmd.qdDes for cmd in commands]))
        pDes = np.concatenate(([cmd.pDes for cmd in commands]))
        vDes = np.concatenate(([cmd.vDes for cmd in commands]))

        kpCartesian = np.concatenate(([cmd.kpCartesian for cmd in commands]))
        kdCartesian = np.concatenate(([cmd.kdCartesian for cmd in commands]))
        kpJoint = np.concatenate(([cmd.kpJoint for cmd in commands]))
        kdJoint = np.concatenate(([cmd.kdJoint for cmd in commands]))

        return tauff, forceff, qDes, qdDes, pDes, vDes, kpCartesian, kdCartesian, kpJoint, kdJoint


class CheetahSimulator:
    def __init__(self, dt, initial_coordinates):
        self.dt = dt
        self.setup_raisim_env()
        self.cheetah.set_generalized_coordinates(initial_coordinates)

    def setup_raisim_env(self):
        world = raisim.World()
        world.set_time_step(self.dt)

        self.vis = raisim.OgreVis.get()

        # these methods must be called before initApp
        print("set world")
        self.vis.set_world(world)
        self.vis.set_window_size(1800, 1200)
        self.vis.set_default_callbacks()
        self.vis.set_setup_callback(self.setup_callback)
        self.vis.set_anti_aliasing(2)

        # starts self.visualizer thread
        print("init app")
        self.vis.init_app()
        print("add ground")
        # create raisim objects
        ground = world.add_ground()
        ground.set_name("checkerboard")

        # create self.visualizer objects
        print("make checkerboard")
        self.vis.create_graphical_object(ground, dimension=20, name="floor", material="checkerboard_green")

        N = 1
        cheetah_urdf_path = os.path.dirname(os.path.abspath(__file__)) + "/./urdf/mini_cheetah_simple.urdf"

        self.cheetah = world.add_articulated_system(cheetah_urdf_path)
        print("dof:", self.cheetah.get_dof())

        self.vis.create_graphical_object(self.cheetah, name="cheetah1")

        # set camera 
        camera = self.vis.get_camera_man().get_camera()
        camera.set_position(N, -2 - N, 1.5 + N)
        camera.pitch(1.)

    def setup_callback(self):
        print("setup callback")
        self.vis = raisim.OgreVis.get()
        print("setup callback")
        # light
        light = self.vis.get_light()
        light.set_diffuse_color(1, 1, 1)
        light.set_cast_shadows(True)
        light.set_direction(self.normalize([-3., -3., -0.5]))
        self.vis.set_camera_speed(300)

        # load textures
        self.vis.add_resource_directory(self.vis.get_resource_dir() + "/material/checkerboard")
        self.vis.load_material("checkerboard.material")

        # shadow setting
        manager = self.vis.get_scene_manager()
        manager.set_shadow_technique(raisim.ogre.ShadowTechnique.SHADOWTYPE_TEXTURE_ADDITIVE)
        manager.set_shadow_texture_settings(2048, 3)

        # scale related settings!! Please adapt it depending on your map size
        # beyond this distance, shadow disappears
        manager.set_shadow_far_distance(10)
        # size of contact points and contact forces
        self.vis.set_contact_visual_object_size(0.03, 0.6)
        # speed of camera motion in freelook mode
        self.vis.get_camera_man().set_top_speed(5)

    def normalize(self, array):
        return np.asarray(array) / np.linalg.norm(array)

    def run_control(self, controller):
        self.vis.set_control_callback(controller)
	
	# run control loop and record video
        self.vis.start_recording_video("./raisim.mp4")
        self.vis.run()
        self.vis.stop_recording_video_and_save()
        self.vis.close_app()

#######################
# Setup RaiSim Simulator
#######################




#######################
# Controller
#######################

class WBC_MPC_Controller:
	# initialization
    def __init__(self, cheetah_sim, vis, dt):
        self.control_decimation = 1

        self.dt = dt
        self.cheetah_ctrl = Cheetah( robot_filename = "../config/mini-cheetah-defaults.yaml",
                                     user_filename = "../config/mc-mit-ctrl-user-parameters.yaml",
                                     dt = self.dt )
        self.cheetah_sim = cheetah_sim
        self.vis = vis

	# initialize controllers

        self.cmpc = ConvexMPCLocomotion(dt, (int)(30 / (1000 * dt)), self.cheetah_ctrl.userparams)
        self.wbc = LocomotionCtrl(self.cheetah_ctrl.model)
        self.wbc_data = LocomotionCtrlData()

        # initialize cheetah sim control mode
        self.cheetah_sim.set_control_mode(raisim.ControlMode.PD_PLUS_FEEDFORWARD_TORQUE)

    def __call__(self):
        self.control_decimation += 1
        if self.control_decimation % 25000 == 0:
            print("############## stopped recording ##############")
            self.vis.stop_recording_video_and_save()

        if self.control_decimation % 100 != 0:
            return

        # update joint state
        p, v = self.cheetah_sim.get_states()
        q, qd = p[-12:], v[-12:]
        #q = np.zeros(12) 
        #qd = np.zeros(12)
        self.cheetah_ctrl.set_joint_state(q, qd)

        # update cheater state
        base_orientation = self.cheetah_sim.get_world_quaternion(0)
        base_pos = self.cheetah_sim.get_world_position(0)
        base_omega = self.cheetah_sim.get_world_angular_velocity(0)
        base_vel = self.cheetah_sim.get_world_linear_velocity(0)
        base_accel = self.cheetah_sim.get_generalized_forces()[0:3] / self.cheetah_sim.get_masses()[0]
        state = np.concatenate((base_orientation, base_pos, base_omega, base_vel, base_accel)) # [orientation(4), pos(3), omegaBody(3), vBody(3), accel(3)]
        self.cheetah_ctrl.set_cartesian_state(state)
        
        #print('base position:', base_pos)

        # run MPC
        self.cmpc.run(self.cheetah_ctrl.fsm.data)

        #print('desired base position:', self.cmpc.pBody_des, 'desired contact state:', self.cmpc.contact_state)
        #print('desired config', self.cmpc.pBody_des, [self.cmpc.get_pFoot_des(i) for i in range(4)], self.cmpc.pBody_RPY_des)
        print('estimated base position:', self.cheetah_ctrl.stateEstimator.getResult().position, self.cheetah_ctrl.stateEstimator.getResult().rpy)
        
    
        self.wbc_data.setBodyDes(self.cmpc.pBody_des, self.cmpc.vBody_des, self.cmpc.aBody_des, self.cmpc.pBody_RPY_des, self.cmpc.vBody_Ori_des)
        for i in range(4):
           self.wbc_data.setFootDes(i, self.cmpc.get_pFoot_des(i), self.cmpc.get_vFoot_des(i), self.cmpc.get_aFoot_des(i), self.cmpc.get_Fr_des(i))
        self.wbc_data.setContactState(self.cmpc.contact_state)

        # run WBC
        self.wbc.run(self.wbc_data, self.cheetah_ctrl.fsm.data)

        # get control output
        tauff, forceff, qDes, qdDes, pDes, vDes, kpCartesian, kdCartesian, kpJoint, kdJoint = self.cheetah_ctrl.get_joint_commands()
       
        #qDes = np.concatenate((qDes[9:], qDes[:9]))

        #p_targets = np.pad(q, (7, 0))
        p_targets = np.pad(qDes, (7, 0))
        d_targets = np.pad(qdDes, (6, 0))
        p_gains = np.pad(np.ones(12)*10, (6, 0))
        d_gains = np.pad(np.ones(12)*1, (6, 0))
        #p_gains = np.zeros(18)
        #d_gains = np.zeros(18)
        generalized_feedforward_forces = np.pad(tauff, (6, 0)) / 10

        #print('q_cur', q)
        #print('q_des', p_targets, 'qd_des', d_targets)

        self.cheetah_sim.set_pd_targets(p_targets, d_targets)
        #print(np.pad(kpJoint, (6, 0)))
        self.cheetah_sim.set_pd_gains(p_gains, d_gains)
        #self.cheetah_sim.set_pd_gains(np.pad(kpJoint, (6, 0)), np.pad(kdJoint, (6, 0)))
        #self.cheetah_sim.set_generalized_forces(generalized_feedforward_forces)



##########################
# Run everything
#########################
dt = 0.025#
#initial_coordinates = [0.0, 0.0, 0.2067, 1.0, 0.0, 0.0, 0.0, -0.21, -0.78, 1.875, 0.21, -0.78, 1.875, -0.275, -0.805, 1.954, 0.275, -0.805, 1.954]

initial_coordinates = [0.0, 0.0, 0.31, 1.0, 0.0, 0.0, 0.0, 0.05694567, -0.7950611, 1.6200384, 0.13700844, -0.6039231, 1.406022, -0.17433, -0.64971113, 1.5169326, -0.09441409, -0.82649213, 1.673078]

print("Initializing simulator...")
simulator = CheetahSimulator(dt/100, initial_coordinates)
print("Simulator ready!")

print("Initializing controller...")
controller = WBC_MPC_Controller(simulator.cheetah, simulator.vis, dt)
print("Controller ready!")

print("Running control loop...")
simulator.run_control(controller)

