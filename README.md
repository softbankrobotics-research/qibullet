# qiBullet [![Build Status](https://api.travis-ci.org/ProtolabSBRE/qibullet.svg?branch=master)](https://travis-ci.org/ProtolabSBRE/qibullet) [![codecov](https://codecov.io/gh/ProtolabSBRE/qibullet/branch/master/graph/badge.svg)](https://codecov.io/gh/ProtolabSBRE/qibullet) [![pypi](https://img.shields.io/pypi/v/qibullet.svg)](https://pypi.org/project/qibullet/) [![Downloads](https://pepy.tech/badge/qibullet)](https://pepy.tech/project/qibullet) [![Gitter chat](https://badges.gitter.im/qibullet.png)](https://gitter.im/qibullet "Gitter chat")

__Bullet-based__ python simulation for __SoftBank Robotics'__ robots.

<!-- start -->
<p align="middle">
	<img src="ressources/short_top_cam.gif" width="34.2%" />
	<img src="ressources/pepper_depth_camera.gif" width="36.5%" />
	<img src="ressources/pepper_moveTo.gif" width="27%" />
</p>
<p align="middle">
	<img src="ressources/nao_walk.gif" width="31%" />
	<img src="ressources/ros_compat.gif" width="68%" />
</p>
<!-- end -->

## Installation

The following modules are required:
* __numpy__
* __pybullet__

The __qiBullet__ module can be installed via pip, for python 2.7 and python 3:
```bash
pip install --user qibullet
```

## Usage
A robot can be spawned via the SimulationManager class:
```python
from qibullet import SimulationManager

if __name__ == "__main__":
    simulation_manager = SimulationManager()

    # Launch a simulation instances, with using a graphical interface.
    # Please note that only one graphical interface can be launched at a time
    client_id = simulation_manager.launchSimulation(gui=True)

    # Selection of the robot type to spawn (True : Pepper, False : NAO)
    pepper_robot = True

    if pepper_robot:
      # Spawning a virtual Pepper robot, at the origin of the WORLD frame, and a
      # ground plane
      pepper = simulation_manager.spawnPepper(
          client_id,
          translation=[0, 0, 0],
          quaternion=[0, 0, 0, 1],
          spawn_ground_plane=True)
    else:
      # Or a NAO robot, at a default position
      nao = simulation_manager.spawnNao(
          client_id,
          spawn_ground_plane=True)
```

Or using loadRobot from the PepperVirtual class if you already have a simulated environment:
```python
    pepper = PepperVirtual()

    pepper.loadRobot(
      translation=[0, 0, 0],
      quaternion=[0, 0, 0, 1],
      physicsClientId=client_id)
```

More snippets can be found in the [examples folder](https://github.com/ProtolabSBRE/qibullet/tree/master/examples), or in the repository [wiki](https://github.com/ProtolabSBRE/qibullet/wiki)

## Documentation
The qiBullet __API documentation__ can be found [here](https://protolabsbre.github.io/qibullet/api/). The documentation can be generated via the following command (the __doxygen__ package has to be installed beforehand, and the docs folder has to exist):
```bash
cd docs
doxygen
```

The repository also contains a [wiki](https://github.com/ProtolabSBRE/qibullet/wiki), providing some tutorials.

## Citations
Please cite qiBullet if you use this repository in your publications:
```
@article{busy2019qibullet,
  title={qiBullet, a Bullet-based simulator for the Pepper and NAO robots},
  author={Busy, Maxime and Caniot, Maxime},
  journal={arXiv preprint arXiv:1909.00779},
  year={2019}
}
```

## Troubleshooting

### OpenGL driver
If you encounter the message:
> Workaround for some crash in the Intel OpenGL driver on Linux/Ubuntu

Your computer is using the Intel OpenGL driver. Go to __Software & Updates__, __Additional Drivers__, and select a driver corresponding to your GPU.

## License
Licensed under the [Apache-2.0 License](LICENSE)
