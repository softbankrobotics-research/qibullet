#!/usr/bin/env python
# coding: utf-8

import sys
import rospy
import pybullet
import pybullet_data
from qibullet import NaoVirtual
from qibullet import RomeoVirtual
from qibullet import PepperVirtual
from qibullet import NaoRosWrapper
from qibullet import RomeoRosWrapper
from qibullet import PepperRosWrapper
from qibullet import SimulationManager

if __name__ == "__main__":
    simulation_manager = SimulationManager()

    if (sys.version_info > (3, 0)):
        rob = input("Which robot should be spawned? (pepper/nao/romeo): ")
    else:
        rob = raw_input("Which robot should be spawned? (pepper/nao/romeo): ")

    client = simulation_manager.launchSimulation(gui=True)

    if rob.lower() == "nao":
        wrap = NaoRosWrapper()
        camera_id = NaoVirtual.ID_CAMERA_TOP
        robot = simulation_manager.spawnNao(client, spawn_ground_plane=True)
    elif rob.lower() == "pepper":
        wrap = PepperRosWrapper()
        camera_id = PepperVirtual.ID_CAMERA_BOTTOM
        robot = simulation_manager.spawnPepper(client, spawn_ground_plane=True)
    elif rob.lower() == "romeo":
        wrap = RomeoRosWrapper()
        camera_id = RomeoVirtual.ID_CAMERA_DEPTH
        robot = simulation_manager.spawnRomeo(client, spawn_ground_plane=True)
    else:
        print("You have to specify a robot, pepper, nao or romeo.")
        simulation_manager.stopSimulation(client)
        sys.exit(1)

    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
    urdf = pybullet.loadURDF(
        "samurai.urdf",
        globalScaling=1.0)

    wrap.launchWrapper(robot, "/naoqi_driver")

    handle = robot.subscribeCamera(camera_id)

    try:
        rospy.spin()

    except KeyboardInterrupt:
        pass
    except rospy.ROSInterruptException:
        pass
    finally:
        wrap.stopWrapper()
        simulation_manager.stopSimulation(client)
