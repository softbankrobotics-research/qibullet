import sys
import time
from qibullet import SimulationManager
from qibullet import PepperVirtual
from qibullet import NaoVirtual
from qibullet import RomeoVirtual


if __name__ == "__main__":
    simulation_manager = SimulationManager()

    if (sys.version_info > (3, 0)):
        rob = input("Which robot should be spawned? (pepper/nao/romeo): ")
    else:
        rob = raw_input("Which robot should be spawned? (pepper/nao/romeo): ")

    client = simulation_manager.launchSimulation(gui=True)

    if rob.lower() == "nao":
        robot = simulation_manager.spawnNao(client, spawn_ground_plane=True)
    elif rob.lower() == "pepper":
        robot = simulation_manager.spawnPepper(client, spawn_ground_plane=True)
    elif rob.lower() == "romeo":
        robot = simulation_manager.spawnRomeo(client, spawn_ground_plane=True)
    else:
        print("You have to specify a robot, pepper, nao or romeo.")
        simulation_manager.stopSimulation(client)
        sys.exit(1)

    # Subscribe to the IMU of the robot with a default frequency
    robot.subscribeImu()
    robot.unsubscribeImu()

    # Get the IMU of the robot as an Imu object
    imu = robot.getImu()
    print("Type of the robot IMU: " + str(type(imu)))

    # Subscribe to the IMU, and define a specific frequency
    robot.subscribeImu(frequency=100)  # Or imu.setFrequency(100)

    try:
        while True:
            # The following method is equivalent to calling
            # imu.getValues()
            angular_velocity, linear_acceleration = robot.getImuValues()

            # One can also retrieve the accelerometer and gyroscope data
            # separately:

            # The following method is equivalent to calling
            # imu.getGyroscopeValues()
            # angular_velocity = robot.getImuGyroscopeValues()

            # The following method is equivalent to calling
            # robot.getImuAccelerometerValues()
            # linear_acceleration = imu.getAccelerometerValues()

            print("Gyroscope values: " + str(angular_velocity))
            print("Accelerometer values: " + str(linear_acceleration))
            time.sleep(1.0)

    except KeyboardInterrupt:
        pass
    finally:
        # Usually, robot.unsubscribeImu() or imu.unsubscribe should be called
        # to stop the IMU data retrieval process. But the stopSimulation method
        # will automatically track and kill active robot module processes
        simulation_manager.stopSimulation(client)
