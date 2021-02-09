#!/usr/bin/env python
# coding: utf-8
import pybullet
import unittest
from qibullet import SimulationManager
from qibullet import NaoVirtual, PepperVirtual, RomeoVirtual


class SimulationManagerTest(unittest.TestCase):
    """
    Unittests for the SimulationManager class
    """

    def test_launch_simulation(self):
        """
        Test the @launchSimulation method
        """
        manager = SimulationManager()

        client = manager.launchSimulation(gui=False)
        self.assertEqual(client, 0)
        manager.stopSimulation(client)

        client = manager.launchSimulation(gui=False, use_shared_memory=True)
        self.assertEqual(client, 0)
        manager.stopSimulation(client)

        client = manager.launchSimulation(
            gui=False,
            use_shared_memory=True,
            auto_step=False)

        self.assertEqual(client, 0)
        manager.stopSimulation(client)

        client = manager.launchSimulation(gui=False, auto_step=False)
        self.assertEqual(client, 0)
        manager.stopSimulation(client)

        # Ensure that stopping the simulation one more time won't raise an
        # error
        try:
            manager.stopSimulation(client)
            self.assertTrue(True)

        except Exception:
            self.assertTrue(False)

    def test_reset_simulation(self):
        """
        Test the @resetSimulation method
        """
        manager = SimulationManager()
        client = manager.launchSimulation(gui=False)
        manager.resetSimulation(client)
        manager.stopSimulation(client)

    def test_stop_simulation(self):
        """
        Test the @stopSimulation method
        """
        manager = SimulationManager()
        client = manager.launchSimulation(gui=False)
        manager.stopSimulation(client)

        with self.assertRaises(pybullet.error):
            pybullet.stepSimulation(physicsClientId=client)

    def test_step_simulation(self):
        """
        Test the @stepSimulation method
        """
        manager = SimulationManager()
        client = manager.launchSimulation(gui=False, auto_step=False)
        manager.stepSimulation(client)

        with self.assertRaises(pybullet.error):
            manager.stepSimulation(client + 1)

        manager.stopSimulation(client)

        client = manager.launchSimulation(
            gui=False,
            use_shared_memory=True,
            auto_step=False)

        manager.stepSimulation(client)

        with self.assertRaises(pybullet.error):
            manager.stepSimulation(client + 1)

        manager.stopSimulation(client)

    def test_get_gravity(self):
        """
        Test the @getGravity method
        """
        manager = SimulationManager()
        client = manager.launchSimulation(gui=False)

        gravity = manager.getGravity(client)
        self.assertIsInstance(gravity, list)
        self.assertEqual(len(gravity), 3)

        # Test with an invalid client
        gravity = manager.getGravity(-1)
        self.assertIsNone(gravity)

        manager.stopSimulation(client)

    def test_set_gravity(self):
        """
        Test the @setGravity method
        """
        manager = SimulationManager()
        client = manager.launchSimulation(gui=False)
        gravities = [
            [0.0, 0.0, -9.81],
            [0.0, 0.0, 9.81],
            [1.0, 3.0, 0.0],
            [-1.0, 3.5, 2.0]]

        for gravity in gravities:
            manager.setGravity(client, gravity)
            value = manager.getGravity(client)

            self.assertIsInstance(value, list)

            for i in range(len(gravity)):
                self.assertEqual(value[i], gravity[i])

        # Test with an invalid client
        try:
            manager.setGravity(-1, gravities[0])
            self.assertTrue(True)

        except Exception:
            self.assertTrue(False)

        manager.stopSimulation(client)

    def test_set_light_position(self):
        """
        Test the @setLightPosition method
        """
        manager = SimulationManager()
        client = manager.launchSimulation(gui=False)

        manager.setLightPosition(client, [10, 20, 2])
        manager.setLightPosition(client, [0, 0, 10])
        manager.setLightPosition(client, [-5, -20, 50])

        with self.assertRaises(pybullet.error):
            manager.setLightPosition(client, "not a list")

        with self.assertRaises(pybullet.error):
            manager.setLightPosition(
                client,
                [1, 2, 3, "wrong amount of elements"])

        manager.stopSimulation(client)

    def test_camera_removal(self):
        """
        Ensure that the camera processes are stopped when removing a robot,
        stopping or resetting a simulation
        """
        manager = SimulationManager()
        client = manager.launchSimulation(gui=False)

        pepper = manager.spawnPepper(
            client,
            spawn_ground_plane=True)
        handle = pepper.subscribeCamera(PepperVirtual.ID_CAMERA_TOP)
        camera = pepper.getCamera(handle)

        self.assertTrue(camera.isActive())
        manager.removePepper(pepper)
        self.assertFalse(camera.isActive())

        pepper = manager.spawnPepper(
            client,
            spawn_ground_plane=True)
        handle = pepper.subscribeCamera(PepperVirtual.ID_CAMERA_TOP)
        camera = pepper.getCamera(handle)

        self.assertTrue(camera.isActive())
        manager.resetSimulation(client)
        self.assertFalse(camera.isActive())

        pepper = manager.spawnPepper(
            client,
            spawn_ground_plane=True)
        handle = pepper.subscribeCamera(PepperVirtual.ID_CAMERA_TOP)
        camera = pepper.getCamera(handle)

        self.assertTrue(camera.isActive())
        manager.stopSimulation(client)
        self.assertFalse(camera.isActive())

    def test_spawn_pepper(self):
        """
        Test the @spawnPepper method
        """
        manager = SimulationManager()
        client = manager.launchSimulation(gui=False)
        pepper = manager.spawnPepper(
            client,
            translation=[0, 0, 0],
            quaternion=[0, 0, 0, 1],
            spawn_ground_plane=True)

        self.assertIsInstance(pepper, PepperVirtual)
        self.assertNotEqual(len(pepper.joint_dict.keys()), 0)
        manager.stopSimulation(client)

    def test_spawn_nao(self):
        """
        Test the @spawnNao method
        """
        manager = SimulationManager()
        client = manager.launchSimulation(gui=False)
        nao = manager.spawnNao(
            client,
            translation=[2, 4, 1],
            quaternion=[0, 0.1, 0, 1],
            spawn_ground_plane=True)

        self.assertIsInstance(nao, NaoVirtual)
        self.assertNotEqual(len(nao.joint_dict.keys()), 0)
        manager.stopSimulation(client)

    def test_spawn_romeo(self):
        """
        Test the @spawnRomeo method
        """
        manager = SimulationManager()
        client = manager.launchSimulation(gui=False)
        romeo = manager.spawnRomeo(
            client,
            translation=[1, 1, 4],
            quaternion=[0, 0, 0.4, 1],
            spawn_ground_plane=True)

        self.assertIsInstance(romeo, RomeoVirtual)
        self.assertNotEqual(len(romeo.joint_dict.keys()), 0)
        manager.stopSimulation(client)

    def test_remove_pepper(self):
        """
        Test the @removePepper method
        """
        manager = SimulationManager()
        client = manager.launchSimulation(gui=False)
        pepper = manager.spawnPepper(client, spawn_ground_plane=True)

        manager.removePepper(pepper)

        with self.assertRaises(pybullet.error):
            pybullet.getBodyInfo(pepper.getRobotModel())

        manager.stopSimulation(client)

    def test_remove_nao(self):
        """
        Test the @removeNao method
        """
        manager = SimulationManager()
        client = manager.launchSimulation(gui=False)
        nao = manager.spawnNao(client, spawn_ground_plane=True)

        manager.removeNao(nao)

        with self.assertRaises(pybullet.error):
            pybullet.getBodyInfo(nao.getRobotModel())

        manager.stopSimulation(client)

    def test_remove_romeo(self):
        """
        Test the @removeRomeo method
        """
        manager = SimulationManager()
        client = manager.launchSimulation(gui=False)
        romeo = manager.spawnRomeo(client, spawn_ground_plane=True)

        manager.removeRomeo(romeo)

        with self.assertRaises(pybullet.error):
            pybullet.getBodyInfo(romeo.getRobotModel())

        manager.stopSimulation(client)


if __name__ == "__main__":
    unittest.main()
