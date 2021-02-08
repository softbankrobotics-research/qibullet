#!/usr/bin/env python
# coding: utf-8

import cv2
import time
from qibullet import SimulationManager
from qibullet import PepperVirtual
from qibullet import Camera


def main():
    simulation_manager = SimulationManager()
    client = simulation_manager.launchSimulation(gui=True)
    pepper = simulation_manager.spawnPepper(client, spawn_ground_plane=True)

    pepper.goToPosture("Crouch", 0.6)
    time.sleep(1)
    pepper.goToPosture("Stand", 0.6)
    time.sleep(1)
    pepper.goToPosture("StandZero", 0.6)
    time.sleep(1)

    # Subscribe to the top RGB camera in QVGA (default value, specified here
    # for more clarity) and 15fps
    handle_top = pepper.subscribeCamera(
        PepperVirtual.ID_CAMERA_TOP,
        resolution=Camera.K_QVGA,
        fps=15.0)

    # Same process for the bottom camera
    handle_bottom = pepper.subscribeCamera(
        PepperVirtual.ID_CAMERA_BOTTOM,
        resolution=Camera.K_QVGA,
        fps=15.0)

    try:
        while True:
            img_top = pepper.getCameraFrame(handle_top)
            img_bottom = pepper.getCameraFrame(handle_bottom)
            cv2.imshow("top camera", img_top)
            cv2.imshow("bottom camera", img_bottom)
            cv2.waitKey(1)

    except KeyboardInterrupt:
        pass
    finally:
        # No need to manually unsubscribe from the cameras when calling
        # stopSimulation, the method will automatically unsubscribe from the
        # cameras
        simulation_manager.stopSimulation(client)


if __name__ == "__main__":
    main()
