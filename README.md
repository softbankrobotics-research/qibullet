# qibullet

__Bullet-based__ python simulation interface for __SoftBank Robotics'__ robots.

## Installation

The following modules are required:
* __numpy__
* __pybullet__

The __qibullet__ module can be installed via pip, for python 2.7 and python 3:
```bash
pip install --user qibullet
```

## Usage

The following snippet demonstrates a simple use of the qibullet module:
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
    pepper.loadRobot([0, 0, 0], [0, 0, 0, 1])

    joint_parameters = list()
    pepper.subscribeCamera(PepperVirtual.ID_CAMERA_BOTTOM)

    # Retrieving an image that can be displayed via an imshow function from
    # openCV
    img = pepper.getCameraFrame()

    while True:
        # Setting joint angles
        pepper.setAngles("HeadYaw", 0, 1.0)
        time.sleep(0.2)
        pepper.setAngles("HeadPitch", 1, 0.6)
        time.sleep(0.2)
        pepper.setAngles(["HeadPitch", "HeadYaw"], [0, 1], 0.6)

        # Moving the robot's base
        pepper.moveTo(1, 1, 0, frame=PepperVirtual.FRAME_ROBOT)
        pepper.moveTo(-1, -1, 0, frame=PepperVirtual.FRAME_ROBOT)

        # Checking for self collisions
        pepper.isSelfColliding('l_wrist')
        pepper.isSelfColliding(["RForeArm", "LForeArm"])
```

More examples can be found in the examples folder

<!-- ![version](https://img.shields.io/badge/status-dev-orange.svg) -->
