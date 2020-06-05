import numpy as np

import pycheetah
from pycheetah import * #FloatingBaseModel, ControlFSMData, LocomotionCtrl, LocomotionCtrlData, PeriodicTaskManager, RobotRunner, MIT_Controller

#fb = FloatingBaseModel()
#fb.addGroundContactPoint(0, np.array([1., 1., 1.]), False)
#fb.addBase(1.0, np.ones(3), np.eye(3))
#print(fb)

cheetah = buildMiniCheetah()
model = cheetah.buildModel()


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
fsm = ControlFSM(model, StateEstimatorContainer(), LegController(), GaitScheduler(), DesiredStateCommand(), RobotControlParameters(), VisualizationData(), MIT_UserParameters())

#lc.run(lc_data, fsm_data)

# RobotRunner

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

