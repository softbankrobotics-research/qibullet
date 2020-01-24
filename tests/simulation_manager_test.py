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

    def test_reset_simulation(self):
        """
        Test the @resetSimulation method
        """
        manager = SimulationManager()
        client = manager.launchSimulation(gui=False)
        manager.resetSimulation(client)
        manager.stopSimulation(client)

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
