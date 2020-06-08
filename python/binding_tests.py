import numpy as np

import pycheetah
from pycheetah import * #FloatingBaseModel, ControlFSMData, LocomotionCtrl, LocomotionCtrlData, PeriodicTaskManager, RobotRunner, MIT_Controller

#fb = FloatingBaseModel()
#fb.addGroundContactPoint(0, np.array([1., 1., 1.]), False)
#fb.addBase(1.0, np.ones(3), np.eye(3))
#print(fb)

# define parameters
robotparams = RobotControlParameters()
robotparams.initializeFromYamlFile("../config/mini-cheetah-defaults.yaml")
userparams = MIT_UserParameters()
userparams.initializeFromYamlFile("../config/mc-mit-ctrl-user-parameters.yaml")
print(userparams.printToYamlString())
print("ok")

# set current state
position = np.array([0.0, 0.0, 0.29])
orientation = np.array([1.0, 0.0, 0.0, 0.0])
vBody = np.array([0.0, 0.0, 0.0])
omegaBody = np.array([0.0, 0.0, 0.0])
acceleration = np.array([0.0, 0.0, 0.0])

# sensor data
accelerometer = np.array([0.0, 0.0, 0.0])
gyro = np.array([0.0, 0.0, 0.0])
quat = orientation

# make model
cheetah = buildMiniCheetah()
model = cheetah.buildModel()

# make state estimator
cheaterState = CheaterState()
cheaterState.orientation, cheaterState.position, cheaterState.omegaBody, cheaterState.vBody, cheaterState.acceleration = orientation, position, omegaBody, vBody, acceleration
vnavData = VectorNavData()
vnavData.accelerometer, vnavData.gyro, vnavData.quat = accelerometer, gyro, quat
legControllerData = LegControllerData()
stateEstimate = StateEstimate()
stateEstimator = StateEstimatorContainer(cheaterState, vnavData, legControllerData, stateEstimate, robotparams)


# test WBC

lc = LocomotionCtrl(model)
lc_data = LocomotionCtrlData()
#print(lc_data.vBody_des)i
pBody_des = np.array([0.0, 0.0, 0.29])
vBody_des, aBody_des, pBody_RPY_des, vBody_Ori_des = np.zeros(3), np.zeros(3), np.zeros(3) , np.zeros(3) 

pFoot_des = np.array([[0.2, -0.15, 0.], [0.2, 0.15, 0.], [-0.2, -0.15, 0.], [-0.2, 0.15, 0.]])
vFoot_des, aFoot_des, Fr_des = np.zeros((4, 3)), np.zeros((4, 3)), np.zeros((4, 3))
contact_state = np.array([1, 1, 1, 1])

lc_data.setBodyDes(pBody_des, vBody_des, aBody_des, pBody_RPY_des, vBody_Ori_des)
for i in range(4):
    lc_data.setFootDes(i, pFoot_des[i], vFoot_des[i], aFoot_des[i], Fr_des[i])
lc_data.setContactState(contact_state)

#fsm_data = ControlFSMData()
#print(fsm_data)

# set command
gamepadCmd = GamepadCommand()
rc_command = rc_control_settings()
# make control FSM
legController = LegController(cheetah)
dt = 0.025
gaitScheduler = GaitScheduler(userparams, dt)
desiredStateCmd = DesiredStateCommand(gamepadCmd, rc_command, robotparams, stateEstimate, dt)
vizData = VisualizationData()
print("[python] Make FSM")
fsm = ControlFSM(cheetah, stateEstimator, legController, gaitScheduler, desiredStateCmd, robotparams, vizData, userparams)
fsm.initialize()
fsm.runFSM()
fsm.printInfo(1)
print("[python] Run WBC")
print(fsm.data)
print(lc_data)
print(lc.run)
#lc.run()
#lc.run(lc_data, fsm.data)
lc.run2(lc_data, fsm.data)

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
