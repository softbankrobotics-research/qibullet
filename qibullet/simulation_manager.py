#!/usr/bin/env python
# coding: utf-8

import time
import pybullet
import threading
import pybullet_data
from qibullet.laser import Laser
from qibullet.camera import Camera
from qibullet.nao_virtual import NaoVirtual
from qibullet.romeo_virtual import RomeoVirtual
from qibullet.pepper_virtual import PepperVirtual
from qibullet.robot_module import RobotModule


class SimulationManager:
    """
    Class allowing to handle the different parameters of a pybullet simulation
    """

    def __init__(self):
        """
        Constructor
        """
        pass

    def launchSimulation(self, gui=True):
        """
        Launches a simulation instance

        Parameters:
            gui - Boolean, if True the simulation is launched with a GUI, and
            with no GUI otherwise

        Returns:
            physics_client - The id of the simulation client created
        """
        if gui:
            physics_client = pybullet.connect(pybullet.GUI)
            pybullet.setRealTimeSimulation(1, physicsClientId=physics_client)
            pybullet.configureDebugVisualizer(
                pybullet.COV_ENABLE_RGB_BUFFER_PREVIEW,
                0,
                physicsClientId=physics_client)
            pybullet.configureDebugVisualizer(
                pybullet.COV_ENABLE_DEPTH_BUFFER_PREVIEW,
                0,
                physicsClientId=physics_client)
            pybullet.configureDebugVisualizer(
                pybullet.COV_ENABLE_SEGMENTATION_MARK_PREVIEW,
                0,
                physicsClientId=physics_client)
        else:
            physics_client = pybullet.connect(pybullet.DIRECT)
            threading.Thread(
                target=self._stepSimulation,
                args=[physics_client]).start()

        pybullet.setGravity(0, 0, -9.81, physicsClientId=physics_client)
        return physics_client

    def resetSimulation(self, physics_client):
        """
        Resets the simulated instance corresponding to the physics client id.
        All of the objects loaded in the simulation will be destroyed, but the
        instance will still be running
        """
        self._clearInstance(physics_client)
        pybullet.resetSimulation(physicsClientId=physics_client)

    def stopSimulation(self, physics_client):
        """
        Stops the simulated instance corresponding to the physics_client id

        Parameters:
            physics_client - The id of the simulated instance to be stopped
        """
        self._clearInstance(physics_client)
        pybullet.disconnect(physicsClientId=physics_client)

    def setLightPosition(self, physics_client, light_position):
        """
        Set the position of the GUI's light (does not work in DIRECT mode)

        Parameters:
            light_position - List containing the 3D positions [x, y, z] along
            the X, Y, and Z axis in the world frame, in meters
        """
        try:
            assert isinstance(light_position, list)
            assert len(light_position) == 3

            pybullet.configureDebugVisualizer(
                lightPosition=light_position,
                physicsClientId=physics_client)

        except AssertionError:
            raise pybullet.error("Incorrect light position format")

    def spawnPepper(
            self,
            physics_client,
            translation=[0, 0, 0],
            quaternion=[0, 0, 0, 1],
            spawn_ground_plane=False):
        """
        Loads a Pepper model in the simulation

        Parameters:
            physics_client - The id of the simulated instance in which the
            robot is supposed to be spawned
            translation - List containing 3 elements, the spawning translation
            [x, y, z] in the WORLD frame
            quaternions - List containing 4 elements, the spawning rotation as
            a quaternion [x, y, z, w] in the WORLD frame
            spawn_ground_plane - If True, the pybullet_data ground plane will
            be spawned

        Returns:
            pepper_virtual - A PepperVirtual object, the Pepper simulated
            instance
        """
        pepper_virtual = PepperVirtual()

        if spawn_ground_plane:
            self._spawnGroundPlane(physics_client)

        pepper_virtual.loadRobot(
            translation,
            quaternion,
            physicsClientId=physics_client)

        return pepper_virtual

    def spawnNao(
            self,
            physics_client,
            translation=[0, 0, 0],
            quaternion=[0, 0, 0, 1],
            spawn_ground_plane=False):
        """
        Loads a NAO model in the simulation

        Parameters:
            physics_client - The id of the simulated instance in which the
            robot is supposed to be spawned
            translation - List containing 3 elements, the spawning translation
            [x, y, z] in the WORLD frame
            quaternions - List containing 4 elements, the spawning rotation as
            a quaternion [x, y, z, w] in the WORLD frame
            spawn_ground_plane - If True, the pybullet_data ground plane will
            be spawned

        Returns:
            nao_virtual - A NaoVirtual object, the NAO simulated instance
        """
        nao_virtual = NaoVirtual()

        if spawn_ground_plane:
            self._spawnGroundPlane(physics_client)

        nao_virtual.loadRobot(
            translation,
            quaternion,
            physicsClientId=physics_client)

        return nao_virtual

    def spawnRomeo(
            self,
            physics_client,
            translation=[0, 0, 0],
            quaternion=[0, 0, 0, 1],
            spawn_ground_plane=False):
        """
        Loads a Romeo model in the simulation

        Parameters:
            physics_client - The id of the simulated instance in which the
            robot is supposed to be spawned
            translation - List containing 3 elements, the spawning translation
            [x, y, z] in the WORLD frame
            quaternions - List containing 4 elements, the spawning rotation as
            a quaternion [x, y, z, w] in the WORLD frame
            spawn_ground_plane - If True, the pybullet_data ground plane will
            be spawned

        Returns:
            romeo_virtual - A RomeoVirtual object, the Romeo simulated instance
        """
        romeo_virtual = RomeoVirtual()

        if spawn_ground_plane:
            self._spawnGroundPlane(physics_client)

        romeo_virtual.loadRobot(
            translation,
            quaternion,
            physicsClientId=physics_client)

        return romeo_virtual

    def removePepper(self, pepper_virtual):
        """
        Removes a Pepper from a simulated instance

        Parameters:
            pepper_virtual - The virtual Pepper robot to be removed
        """
        pepper_virtual.laser_manager._terminateModule()
        pepper_virtual.base_controller._terminateModule()
        pepper_virtual.unsubscribeCamera(PepperVirtual.ID_CAMERA_TOP)
        pepper_virtual.unsubscribeCamera(PepperVirtual.ID_CAMERA_BOTTOM)
        pepper_virtual.unsubscribeCamera(PepperVirtual.ID_CAMERA_DEPTH)

        pybullet.removeBody(pepper_virtual.getRobotModel())

    def removeNao(self, nao_virtual):
        """
        Removes a NAO from a simulated instance

        Parameters:
            nao_virtual - The virtual NAO robot to be removed
        """
        nao_virtual.unsubscribeCamera(NaoVirtual.ID_CAMERA_TOP)
        nao_virtual.unsubscribeCamera(NaoVirtual.ID_CAMERA_BOTTOM)

        pybullet.removeBody(nao_virtual.getRobotModel())

    def removeRomeo(self, romeo_virtual):
        """
        Removes a Romeo from a simulated instance

        Parameters:
            romeo_virtual - The virtual Romeo to be removed
        """
        romeo_virtual.unsubscribeCamera(RomeoVirtual.ID_CAMERA_LEFT)
        romeo_virtual.unsubscribeCamera(RomeoVirtual.ID_CAMERA_RIGHT)
        romeo_virtual.unsubscribeCamera(RomeoVirtual.ID_CAMERA_DEPTH)

        pybullet.removeBody(romeo_virtual.getRobotModel())

    def _clearInstance(self, physics_client):
        """
        INTERNAL METHOD, Called to kill the processes of modules running in a
        simulated instance, before resetting or stopping it.

        Parameters:
            physics_client - The client id of the simulated instance that will
            be cleared
        """
        Camera.ACTIVE_OBJECT_ID[physics_client] = -1

        for module in RobotModule._getInstances():
            if module.getPhysicsClientId() == physics_client:
                module._terminateModule()

    def _stepSimulation(self, physics_client):
        """
        INTERNAL METHOD: This method is only used for a simulation in DIRECT
        mode (without the gui).

        Parameters:
            physics_client - The id of the simulated instance to be stepped
        """
        try:
            while True:
                pybullet.stepSimulation(physicsClientId=physics_client)
                time.sleep(1./240.)
        except Exception:
            pass

    def _spawnGroundPlane(self, physics_client):
        """
        INTERNAL METHOD, Loads a ground plane

        Parameters:
            physics_client - The id of the simulated instance in which the
            ground plane is supposed to be spawned
        """
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        pybullet.loadMJCF(
            "mjcf/ground_plane.xml",
            physicsClientId=physics_client)
