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
        for value in LinkTest.robot.link_dict.values():
            index = value.index
            name = value.name
            parent_index = value.parent_index

            self.assertEqual(index, value.getIndex())
            self.assertEqual(name, value.getName())
            self.assertEqual(parent_index, value.getParentIndex())

        # Test Link equality
        self.assertEqual(
            list(LinkTest.robot.link_dict.values())[0],
            list(LinkTest.robot.link_dict.values())[0])

        self.assertNotEqual(
            list(LinkTest.robot.link_dict.values())[0],
            list(LinkTest.robot.link_dict.values())[1])

    def test_get_link(self):
        """
        Test the @getLink method
        """
        for key, value in LinkTest.robot.link_dict.items():
            self.assertEqual(LinkTest.robot.getLink(key), value)

        with self.assertRaises(KeyError):
            LinkTest.robot.getLink("non_existent_link")

    def test_get_link_position(self):
        """
        Test the @getLinkPosition method
        """
        # Only test that the values returned by getLinkPosition are indeed a
        # 3D translation and a 4D orientation (quaternion)
        translation, orientation = LinkTest.robot.getLinkPosition("torso")
        self.assertEqual(len(translation), 3)
        self.assertEqual(len(orientation), 4)

        with self.assertRaises(KeyError):
            LinkTest.robot.getLinkPosition("non_existent_link")


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

    def test_get_link(self):
        LinkTest.test_get_link(self)

    def test_get_link_position(self):
        LinkTest.test_get_link_position(self)


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

    def test_get_link(self):
        LinkTest.test_get_link(self)

    def test_get_link_position(self):
        LinkTest.test_get_link_position(self)


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

    def test_get_link(self):
        LinkTest.test_get_link(self)

    def test_get_link_position(self):
        LinkTest.test_get_link_position(self)


if __name__ == "__main__":
    unittest.main()
