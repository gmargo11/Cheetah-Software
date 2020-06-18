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
export PYTHONPATH=$PYTHONPATH:$LOCAL_BUILD/lib
ldconfig -v
```

Then, in python:
```python
import pycheetah
```

### Testing the Bindings

There are simple tests for the python bindings in `python/binding_tests.py`.

### Running the Controller with RaiSim

You will need to install RaiSim and the Raisimpy python bindings (https://github.com/robotlearn/raisimpy).

Then, run the python script `python/raisim_control.py` which executes the MIT Controller on the mini cheetah in RaiSim. The video is saved as `video/raisim.mp4`.

In RaiSim, you can easily generate custom terrains, simulate objects, adjust surface properties, and more.

### Adjusting the Gait Parameters

The NeuralGaitMPC controller, based on the VisionMPC controller, accepts the gait parameters (offset, duration, velocity) as an argument to its `run` function. You can see an example in `python/raisim_control_adaptive_gait.py`.
