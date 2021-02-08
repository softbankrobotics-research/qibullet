import time
from qibullet import SimulationManager
from qibullet import NaoVirtual
from qibullet import NaoFsr


if __name__ == "__main__":
    simulation_manager = SimulationManager()
    client = simulation_manager.launchSimulation(gui=True)
    nao = simulation_manager.spawnNao(client, spawn_ground_plane=True)

    # Alternative solution, get the FsrHandler of the robot:
    # fsr_handler = nao.getFsrHandler

    try:
        while True:
            # Get the FSR value of the front left FSR of NAO's left foot
            value = nao.getFsrValue(NaoFsr.LFOOT_FL)
            # Get the FSR value of the rear right FSR of NAO's right foot
            value = nao.getFsrValue(NaoFsr.RFOOT_RR)

            # Get all of the values of the FSRs of NAO's left foot
            values = nao.getFsrValues(NaoFsr.LFOOT)

            # Get the total weight value on the FSRs of NAO's right foot
            total_weight = nao.getTotalFsrValues(NaoFsr.RFOOT)
            print("Total weight on the right foot: " + str(total_weight))

            # Alternative solution:
            # fsr_handler.getValue(NaoFsr.LFOOT_FL)
            # fsr_handler.getValue(NaoFsr.RFOOT_RR)
            # fsr_handler.getValues(NaoFsr.LFOOT)
            # fsr_handler.getTotalValue(NaoFsr.RFOOT)

            time.sleep(0.5)

    except KeyboardInterrupt:
        pass
    finally:
        simulation_manager.stopSimulation(client)
