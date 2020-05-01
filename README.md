## Cheetah-Software: Gabe's Fork

This repository is forked from mit-biomimetics/Cheetah-Software.

The first order of business was to make the simulator run headless so we can use it more easily via docker + ssh. 

To run a headless sim:
```
export QT_QPA_PLATFORM="offscreen"
cd mc-build
./sim/sim
```

TODO:
- ffmpeg pipeline to download headless sim results
- Demonstrate hardcoded forward walk in sim
- Send direction, velocity commands from python via lcm
- ...
- Send footstep commands via LCM
