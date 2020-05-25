## Cheetah-Software: Gabe's Fork

This repository is forked from mit-biomimetics/Cheetah-Software.

### Running the Simulator and Controller
The first order of business was to make the simulator run headless so we can use it more easily via docker + ssh. 

To run a headless sim:
```
cd mc-build
xvfb-run -a --server-args="-screen 0, 1280x1024x24" ./sim/sim
```
and start the controller in a separate terminal:
```
./user/MIT_Controller/mit_ctrl m s
```

### Configuration & Python LCM Interface 

Parameters are loaded by the simulator from `config/mc-mit-ctrl-user-parameters.yaml`. Initial state is specified in Simulation.cpp (lines 92...150). 

Send updates to parameters from python using pycheetah (https://github.com/Improbable-AI/locomotion/tree/master/mini_cheetah/pycheetah)

### Neural Locomotion State

The neural controller is implemented as state 12 of the MIT_Controller FSM. Using pycheetah, transitioning to neural control mode is as simple as running the Simulator, Controller, and then executing the python script:
```python
cheetah = CheetahInterface()
cheetah.set_FSM_state(12)
```
