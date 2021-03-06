## Cheetah-Software: Gabe's Fork

This repository is forked from mit-biomimetics/Cheetah-Software.

### Building

```bash
cd Cheetah-Software
mkdir mc-build && cd mc-build
cmake -DPYTHON_EXECUTABLE:FILEPATH=/usr/bin/python3.6 ..
make -j4
make install
```

### Python Binding Usage

```bash
ldconfig -v
export PYTHONPATH=$PYTHONPATH:$LOCAL_BUILD/lib
```

Then, in python:
```python
import pycheetah
```

### Testing the Bindings

There are simple tests for the python bindings in `python/binding_tests.py`.

### Interfacing the Controller with RaiSim

You will need to install RaiSim and the Raisimpy python bindings (https://github.com/robotlearn/raisimpy).

Then, run the python script `python/raisim_control.py` which executes the MIT Controller on the mini cheetah in RaiSim. The video is saved as `video/raisim.mp4`.

[Here's a video](https://drive.google.com/file/d/1W2t1wIL4ATKPf3ZOJo1CU6EcOaPUB7og/view?usp=sharing) of mini-cheetah trotting on rough terrain in RaiSim using Convex MPC and WBIC.

If you are running on a remote server, you may want to use xvfb to generate video headlessly: `xvfb-run -a --server-args="-screen 0, 1280x1024x24" python3 raisim_control.py`

In RaiSim, you can easily generate custom terrains, simulate objects, adjust surface properties, and more.

### Adjusting the Gait Parameters

The NeuralGaitMPC controller is under construction. Currently, it accepts the gait parameters (offset, duration, velocity) as an argument to its `run` function. You can see an example usage of the python bindings in `python/raisim_control_adaptive_gait.py`.

[Here's a video](https://drive.google.com/file/d/1Pgiw8QcwIuaW4sZ0dg2KvlUhbQsKotGU/view?usp=sharing) of mini-cheetah making a preprogrammed gait switch on rough terrain.
