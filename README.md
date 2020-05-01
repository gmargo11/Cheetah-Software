## Cheetah-Software: Gabe's Fork

This repository is forked from mit-biomimetics/Cheetah-Software.

The first order of business was to make the simulator run headless so we can use it more easily via docker + ssh. 

To run a headless sim:
```
export QT_QPA_PLATFORM="offscreen"
cd mc-build
./sim/sim
```
and start the controller in a separate terminal:
```
./user/MIT_Controller/mit_ctrl m s
```

Parameters are loaded by the simulator from `config/mc-mit-ctrl-user-parameters.yaml`. Initial state is specified in Simulation.cpp (lines 92...150). 



TODO:
- ffmpeg pipeline to download headless sim results
- Demonstrate hardcoded forward walk in sim
- Send direction, velocity commands from python via lcm
- ...
- Send footstep commands via LCM
