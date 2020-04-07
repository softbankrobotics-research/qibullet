#!/usr/bin/env python
# coding: utf-8

import cv2
import time
from qibullet import SimulationManager
from qibullet import PepperVirtual


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
    handle = pepper.subscribeCamera(PepperVirtual.ID_CAMERA_BOTTOM)

    try:
        while True:
            img = pepper.getCameraFrame(handle)
            cv2.imshow("bottom camera", img)
            cv2.waitKey(1)

    except KeyboardInterrupt:
        simulation_manager.stopSimulation(client)


if __name__ == "__main__":
    main()
