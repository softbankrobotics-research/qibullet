# qibullet [![Build Status](https://api.travis-ci.org/ProtolabSBRE/qibullet.svg?branch=master)](https://travis-ci.org/ProtolabSBRE/qibullet) [![pypi](https://img.shields.io/pypi/v/qibullet.svg)](https://pypi.org/project/qibullet/)

__Bullet-based__ python simulation for __SoftBank Robotics'__ robots.

## Installation

The following modules are required:
* __numpy__
* __pybullet__

The __qibullet__ module can be installed via pip, for python 2.7 and python 3:
```bash
pip install --user qibullet
```

## Usage
Please note that only the Pepper robot is currently handled by this module. A robot can be spawn via the SimulationManager class:
```python
from qibullet import SimulationManager

if __name__ == "__main__":
    simulation_manager = SimulationManager()

    # Launch a simulation instances, with using a graphical interface.
    # Please note that only one graphical interface can be launched at a time
    client_id = simulation_manager.launchSimulation(gui=True)

    # Spawning a virtual Pepper robot, at the origin of the WORLD frame
    pepper = simulation_manager.spawnPepper(
        client_id,
        [0, 0, 0],
        [0, 0, 0, 1],
        spawn_ground_plane=True)
```

Or via the PepperVirtual class:
```python
import time
import pybullet as p
import pybullet_data
from qibullet import PepperVirtual

if __name__ == "__main__":
    physicsClient = p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)

    p.setRealTimeSimulation(1)
    p.setGravity(0, 0, -9.81)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadMJCF("mjcf/ground_plane.xml")

    pepper = PepperVirtual()
    pepper.loadRobot([0, 0, 0], [0, 0, 0, 1], physicsClientId=physicsClient)
```

More snippets can be found in the examples folder:
* [A basic usage of the PepperVirtual class](examples/pepper_basic.py)
* [A "joint debug display", letting the user change joint's positions with cursors](examples/pepper_joint_control.py)
* [Launch multiple simulation instances](examples/multi_simulation.py)
* [A script using several simulation instances to compute the error on the joint positions](examples/pepper_joints_error.py)
* [A basic usage of the PepperRosWrapper class (bridge between qibullet and ROS)](examples/pepper_ros_test.py)
* [Script allowing a simulated model to mimic a real robot's movements](examples/pepper_shadowing.py)

## Documentation
The qibullet __API documentation__ can be found [here](https://protolabsbre.github.io/qibullet/api/). The documentation can be generated via the following command (the __doxygen__ package has to be installed beforehand, and the docs folder has to exist):
```bash
cd docs
doxygen
```

## Troubleshooting

### OpenGL driver
If you encounter the message:
> Workaround for some crash in the Intel OpenGL driver on Linux/Ubuntu

Your computer is using the Intel OpenGL driver. Go to __Software & Updates__, __Additional Drivers__, and select a driver corresponding to your GPU.
