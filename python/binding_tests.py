import numpy as np

import pycheetah
from pycheetah import FloatingBaseModel, ControlFSMData, LocomotionCtrl, LocomotionCtrlData, PeriodicTaskManager, RobotRunner, MIT_Controller, buildMiniCheetah

#fb = FloatingBaseModel()
#fb.addGroundContactPoint(0, np.array([1., 1., 1.]), False)
#fb.addBase(1.0, np.ones(3), np.eye(3))
#print(fb)

cheetah = buildMiniCheetah()
model = cheetah.buildModel()


lc = LocomotionCtrl(model)
lc_data = LocomotionCtrlData()
print(lc_data.vBody_des)

fsm_data = ControlFSMData()
print(fsm_data)

lc.run(lc_data, fsm_data)

# RobotRunner
'''
ctrl = MIT_Controller()
task_manager = PeriodicTaskManager()

runner = RobotRunner(ctrl, task_manager, 0, "robot-task")
user_params = ctrl.getUserControlParameters()

runner.init()
runner.run()
'''
