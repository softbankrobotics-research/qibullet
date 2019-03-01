#!/usr/bin/env python
# coding: utf-8

import random
import pybullet
import pybullet_data
from qibullet import PepperVirtual
from qibullet import SimulationManager


def main():
    simulation_manager = SimulationManager()
    client = simulation_manager.launchSimulation(gui=True)
    pepper = simulation_manager.spawnPepper(client, spawn_ground_plane=True)

    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
    pybullet.loadURDF(
        "sphere2red.urdf",
        basePosition=[0, 2, 0.5],
        globalScaling=1.0,
        physicsClientId=client)
    pybullet.loadURDF(
        "sphere2red.urdf",
        basePosition=[2, 0, 1],
        globalScaling=1.0,
        physicsClientId=client)
    pybullet.loadURDF(
        "duck_vhacd.urdf",
        basePosition=[-1, 0, 1],
        globalScaling=5.0,
        physicsClientId=client)
    pybullet.loadURDF(
        "duck_vhacd.urdf",
        basePosition=[1, -1, 0.5],
        globalScaling=10.0,
        physicsClientId=client)

    pepper.showLaser(True)
    pepper.subscribeLaser()
    pepper.goToPosture("Stand", 0.6)

    text_id = pybullet.addUserDebugText(
        "Laser scan",
        textPosition=[0, 0, 1.4],
        textColorRGB=[0, 0, 1])

    while True:
        laser_list = pepper.getRightLaserValue()
        laser_list.extend(pepper.getFrontLaserValue())
        laser_list.extend(pepper.getLeftLaserValue())

        print laser_list
        if all(laser == 5.6 for laser in laser_list):
            pybullet.addUserDebugText(
                "Nothing detected by the lasers",
                textPosition=[0, 0, 1.4],
                textColorRGB=[1, 0, 0],
                replaceItemUniqueId=text_id)
        else:
            pybullet.addUserDebugText(
                "Something has been detected by the lasers",
                textPosition=[0, 0, 1.4],
                textColorRGB=[0, 1, 0],
                replaceItemUniqueId=text_id)


if __name__ == "__main__":
    main()
