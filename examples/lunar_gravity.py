import sys
import time
from qibullet import SimulationManager
from qibullet import NaoVirtual


if __name__ == "__main__":
    simulation_manager = SimulationManager()
    client = simulation_manager.launchSimulation(gui=True)

    # Set a lunar gravity for the instance. The gravity of the instance can be
    # retrieved with the getGravity method
    simulation_manager.setGravity(client, [0.0, 0.0, -1.62])

    # Spawn a nao robot
    nao = simulation_manager.spawnNao(client, spawn_ground_plane=True)

    if (sys.version_info > (3, 0)):
        input("Press any key to stop the simulation")
    else:
        raw_input("Press any key to stop the simulation")

    simulation_manager.stopSimulation(client)
