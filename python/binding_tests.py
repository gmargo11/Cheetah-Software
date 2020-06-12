import numpy as np
import time

import pycheetah
from pycheetah import * #FloatingBaseModel, ControlFSMData, LocomotionCtrl, LocomotionCtrlData, PeriodicTaskManager, RobotRunner, MIT_Controller

#fb = FloatingBaseModel()
#fb.addGroundContactPoint(0, np.array([1., 1., 1.]), False)
#fb.addBase(1.0, np.ones(3), np.eye(3))
#print(fb)


def load_params(robot_filename, user_filename):
    # define parameters
    robotparams = RobotControlParameters()
    robotparams.initializeFromYamlFile(robot_filename)
    userparams = MIT_UserParameters()
    userparams.initializeFromYamlFile(user_filename)
    #print(userparams.printToYamlString())
    #print("ok")
    return robotparams, userparams

# set current state
position = np.array([0.0, 0.0, 0.29])
orientation = np.array([1.0, 0.0, 0.0, 0.0])
vBody = np.array([0.0, 0.0, 0.0])
omegaBody = np.array([0.0, 0.0, 0.0])
acceleration = np.array([0.0, 0.0, 0.0])

# sensor data
#accelerometer = np.array([0.0, 0.0, 0.0])
#gyro = np.array([0.0, 0.0, 0.0])
#quat = orientation

def make_model():
    # make model
    cheetah = buildMiniCheetah()
    model = cheetah.buildModel()
    return cheetah, model

def make_state_estimator(robotparams):
    # make state estimator
    cheaterState = CheaterState()
    # cheaterState.orientation, cheaterState.position, cheaterState.omegaBody, cheaterState.vBody, cheaterState.acceleration = orientation, position, omegaBody, vBody, acceleration
    vnavData = VectorNavData()
#vnavData.accelerometer, vnavData.gyro, vnavData.quat = accelerometer, gyro, quat
    legControllerData = LegControllerData() # MIGHT NEED TO MAKE THIS NULLPOINTER (?)

    stateEstimate = StateEstimate()
    stateEstimator = StateEstimatorContainer(cheaterState, vnavData, legControllerData, stateEstimate, robotparams)
    stateEstimator.initializeCheater()

    return stateEstimator, cheaterState, legControllerData

def set_cheater_state(state):


# test MPC

# test WBC

#lc = LocomotionCtrl(model)
#lc_data = LocomotionCtrlData()
#print(lc_data.vBody_des)i
#pBody_des = np.array([0.0, 0.0, 0.29])
#vBody_des, aBody_des, pBody_RPY_des, vBody_Ori_des = np.zeros(3), np.zeros(3), np.zeros(3) , np.zeros(3) 

#pFoot_des = np.array([[0.2, -0.15, 0.], [0.2, 0.15, 0.], [-0.2, -0.15, 0.], [-0.2, 0.15, 0.]])
#vFoot_des, aFoot_des, Fr_des = np.zeros((4, 3)), np.zeros((4, 3)), np.zeros((4, 3))
#contact_state = np.array([1, 1, 1, 1])

#lc_data.setBodyDes(pBody_des, vBody_des, aBody_des, pBody_RPY_des, vBody_Ori_des)
#for i in range(4):
#    lc_data.setFootDes(i, pFoot_des[i], vFoot_des[i], aFoot_des[i], Fr_des[i])
#lc_data.setContactState(contact_state)

#fsm_data = ControlFSMData()
#print(fsm_data)

def make_fsm(cheetah, userparams, robotparams):

    # set command
    dt = 0.025
    gamepadCmd = GamepadCommand()
    rc_command = rc_control_settings()
    # make control FSM
    legController = LegController(cheetah)
    gaitScheduler = GaitScheduler(userparams, dt)
    desiredStateCmd = DesiredStateCommand(gamepadCmd, rc_command, robotparams, stateEstimate, dt)
    vizData = VisualizationData()
    print("[python] Make FSM")
    fsm = ControlFSM(cheetah, stateEstimator, legController, gaitScheduler, desiredStateCmd, robotparams, vizData, userparams)
    fsm.initialize()
    fsm.runFSM()
    fsm.printInfo(1)

    return fsm, rc_command, legController


#######################
# Run Controller
#######################


# initialization

userparams, robotparams = load_params(robot_filename = "../config/mini-cheetah-defaults.yaml",
                                      user_filename = "../config/mc-mit-ctrl-user-parameters.yaml")
cheetah, model = make_model()
stateEstimator, cheaterState, legControllerData = make_state_estimator(robotparams)
fsm, rc_command, legController = make_fsm(cheetah, userparams, robotparams)

# initialize controllers

cmpc = ConvexMPCLocomotion(dt, (int)(30 / (1000 * dt)), userparams)
wbc = LocomotionCtrl(model)
wbc_data = LocomotionCtrlData()

# control loop!
tstart = time.time()

iterations = 1000

for i in range(iterations):

    # update joint state
    q = np.zeros(12) 
    qd = np.zeros(12)
    spidata = make_spidata(q, qd)
    legController.updateCommand(spidata)

    # update cheater state
    state = np.zeros(16) # [orientation(4), pos(3), omegaBody(3), vBody(3), accel(3)]
    cheaterState = update_cheaterstate(cheaterState, state)


    # run state estimator
    stateEstimator.run()


    # run MPC
    cmpc.run(fsm.data)

    
    wbc_data.setBodyDes(cmpc.pBody_des, cmpc.vBody_des, cmpc.aBody_des, cmpc.pBody_RPY_des, cmpc.vBody_Ori_des)
    for i in range(4):
        wbc_data.setFootDes(i, cmpc.get_pFoot_des(i), cmpc.get_vFoot_des(i), cmpc.get_aFoot_des(i), cmpc.get_Fr_des(i))
    wbc_data.setContactState(cmpc.contact_state)

    # run WBC
    wbc.run(wbc_data, fsm.data)

    # check final leg command
    commands = [legController.getCommands(i) for i in range(4)]#getCommands()
    #for command in commands:
    print([command.tauFeedForward for command in commands])

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
