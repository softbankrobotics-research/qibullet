#!/usr/bin/env python
# coding: utf-8

import math
import time
from qibullet import SimulationManager
from qibullet import NaoVirtual

if __name__ == "__main__":
    simulation_manager = SimulationManager()
    client = simulation_manager.launchSimulation(gui=True)
    nao = simulation_manager.spawnNao(client, spawn_ground_plane=True)
    r = 20

    for i in range(500):
        angle_rad = (math.pi * (i % 360)) / 180.0
        simulation_manager.setLightPosition(
            client,
            [r * math.cos(angle_rad), r * math.sin(angle_rad), 15])

        time.sleep(0.01)

    simulation_manager.stopSimulation(client)
