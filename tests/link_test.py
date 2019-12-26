#!/usr/bin/env python
# coding: utf-8
import unittest
import pybullet
from qibullet import SimulationManager
from qibullet import NaoVirtual, PepperVirtual, RomeoVirtual


class LinkTest(unittest.TestCase):
    """
    Unittests for the virtual links (virtual class don't use directly)
    """

    def test_links_characteristics(self):
        """
        Test the behaviour of the Joint class with the robot's characteristics
        """
        for key, value in LinkTest.robot.link_dict.items():
            index = value.index
            name = value.name
            parent_index = value.parent_index

            self.assertEqual(key, value.getName())
            self.assertEqual(index, value.getIndex())
            self.assertEqual(name, value.getName())
            self.assertEqual(parent_index, value.getParentIndex())

        # Test Link equality
        self.assertEqual(
            LinkTest.robot.link_dict.values()[0],
            LinkTest.robot.link_dict.values()[0])

        self.assertNotEqual(
            LinkTest.robot.link_dict.values()[0],
            LinkTest.robot.link_dict.values()[1])


class PepperLinkTest(LinkTest):
    """
    Unittests for the control of Pepper virtual's joints
    """

    @classmethod
    def setUpClass(cls):
        """
        Launches a simulation and spawns the Pepper virtual robot
        """
        LinkTest.simulation = SimulationManager()
        LinkTest.client = LinkTest.simulation.launchSimulation(
            gui=False)

        LinkTest.robot = LinkTest.simulation.spawnPepper(
            LinkTest.client,
            spawn_ground_plane=True)

    @classmethod
    def tearDownClass(cls):
        """
        Stops the simulation
        """
        LinkTest.simulation.stopSimulation(
            LinkTest.client)

    def test_links_characteristics(self):
        LinkTest.test_links_characteristics(self)


class NaoLinkTest(LinkTest):
    """
    Unittests for the control of Nao virtual's joints
    """

    @classmethod
    def setUpClass(cls):
        """
        Launches a simulation and spawns the NAO virtual robot
        """
        LinkTest.simulation = SimulationManager()
        LinkTest.client = LinkTest.simulation.launchSimulation(
            gui=False)

        LinkTest.robot = LinkTest.simulation.spawnNao(
            LinkTest.client,
            spawn_ground_plane=True)

    @classmethod
    def tearDownClass(cls):
        """
        Stops the simulation
        """
        LinkTest.simulation.stopSimulation(
            LinkTest.client)

    def test_links_characteristics(self):
        LinkTest.test_links_characteristics(self)


class RomeoLinkTest(LinkTest):
    """
    Unittests for the control of Nao virtual's joints
    """

    @classmethod
    def setUpClass(cls):
        """
        Launches a simulation and spawns the NAO virtual robot
        """
        LinkTest.simulation = SimulationManager()
        LinkTest.client = LinkTest.simulation.launchSimulation(
            gui=False)

        LinkTest.robot = LinkTest.simulation.spawnRomeo(
            LinkTest.client,
            spawn_ground_plane=True)

    @classmethod
    def tearDownClass(cls):
        """
        Stops the simulation
        """
        LinkTest.simulation.stopSimulation(
            LinkTest.client)

    def test_links_characteristics(self):
        LinkTest.test_links_characteristics(self)


if __name__ == "__main__":
    unittest.main()
