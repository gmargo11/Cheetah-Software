import numpy as np
import time

import pycheetah
from pycheetah import * #FloatingBaseModel, ControlFSMData, LocomotionCtrl, LocomotionCtrlData, PeriodicTaskManager, RobotRunner, MIT_Controller

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
        self.fsm.runFSM()
        #self.fsm.printInfo(1)

    def set_cartesian_state(self, cartesian_state):
        self.cheaterState.orientation = state[0:4]
        self.cheaterState.position = state[4:7]
        self.cheaterState.omegaBody = state[7:10]
        self.cheaterState.vBody = state[10:13]
        self.cheaterState.acceleration = state[13:16]

        self.stateEstimator.run()

    def set_joint_state(self, joint_state, d_joint_state):
        spiData = SpiData()
        for idx in range(4):
            q_abad, q_hip, q_knee = joint_state[3 * idx], joint_state[3 * idx + 1], joint_state[3 * idx + 2]
            qd_abad, qd_hip, qd_knee = d_joint_state[3 * idx], d_joint_state[3 * idx + 1], d_joint_state[3 * idx + 2]
            spiData.setLegData(idx, q_abad, q_hip, q_knee, qd_abad, qd_hip, qd_knee)
        self.legController.updateData(spiData)

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


#######################
# Run Controller
#######################


# initialization

dt = 0.025
cheetah_ctrl = Cheetah( robot_filename = "../config/mini-cheetah-defaults.yaml",
                        user_filename = "../config/mc-mit-ctrl-user-parameters.yaml",
                        dt = dt )

# initialize controllers

cmpc = ConvexMPCLocomotion(dt, (int)(30 / (1000 * dt)), cheetah_ctrl.userparams)
wbc = LocomotionCtrl(cheetah_ctrl.model)
wbc_data = LocomotionCtrlData()

# control loop!
tstart = time.time()

iterations = 1000

for i in range(iterations):

    # update joint state
    q = np.zeros(12) 
    qd = np.zeros(12)
    cheetah_ctrl.set_joint_state(q, qd)

    # update cheater state
    state = np.zeros(16) # [orientation(4), pos(3), omegaBody(3), vBody(3), accel(3)]
    cheetah_ctrl.set_cartesian_state(state)


    # run MPC
    cmpc.run(cheetah_ctrl.fsm.data)

    
    wbc_data.setBodyDes(cmpc.pBody_des, cmpc.vBody_des, cmpc.aBody_des, cmpc.pBody_RPY_des, cmpc.vBody_Ori_des)
    for i in range(4):
        wbc_data.setFootDes(i, cmpc.get_pFoot_des(i), cmpc.get_vFoot_des(i), cmpc.get_aFoot_des(i), cmpc.get_Fr_des(i))
    wbc_data.setContactState(cmpc.contact_state)

    # run WBC
    wbc.run(wbc_data, cheetah_ctrl.fsm.data)

    # get control output
    tauff, forceff, qDes, qdDes, pDes, vDes, kpCartesian, kdCartesian, kpJoint, kdJoint = cheetah_ctrl.get_joint_commands()
    print(tauff, forceff, qDes, qdDes)

print("control frequency: ", iterations/(time.time()-tstart))
print("required frequency: ", 1/dt)

# RobotRunner
'''
ctrl = MIT_Controller()
task_manager = PeriodicTaskManager()

#ctrl.initializeController()
#ctrl.runController()

runner = RobotRunner(ctrl, task_manager, 0, "robot-task")

print("ok")

orientation = np.array([1.0, 0.0, 0.0, 0.0])
position = np.array([0.0, 0.0, 0.29])
omegaBody = np.array([0.0, 0.0, 0.0])
vBody = np.array([0.0, 0.0, 0.0])
acceleration = np.array([0.0, 0.0, 0.0])

runner.setupData(orientation, position, omegaBody, vBody, acceleration);
print("setup complete!")


#runner.run(:q:q
ctrl.runController()
print("one run!")
ctrl.runController()
print("two runs!")


#user_params = ctrl.getUserControlParameters()

#runner.init()
#runner.run()
'''
