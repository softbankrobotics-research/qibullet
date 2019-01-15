#!/usr/bin/env python
# coding: utf-8

import time
from qibullet import SimulationManager

if __name__ == "__main__":
    simulation_manager = SimulationManager()
    simulation_a = simulation_manager.launchSimulation(gui=False)
    simulation_b = simulation_manager.launchSimulation(gui=False)

    simulation_manager.spawnPepper(
        simulation_a,
        [0, 0, 0],
        [0, 0, 0, 1],
        spawn_ground_plane=True)
    simulation_manager.spawnPepper(
        simulation_b,
        [0, 0, 0],
        [0, 0, 0, 1],
        spawn_ground_plane=True)

    time.sleep(2)

    simulation_manager.resetSimulation(simulation_a)
    simulation_manager.resetSimulation(simulation_b)

    time.sleep(2)

    simulation_manager.stopSimulation(simulation_a)
    simulation_manager.stopSimulation(simulation_b)
