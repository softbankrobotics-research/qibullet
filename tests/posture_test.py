#!/usr/bin/env python
# coding: utf-8
import unittest
from qibullet import SimulationManager
from qibullet import NaoVirtual, PepperVirtual


class PostureTest(unittest.TestCase):
    """
    Unittests for Pepper's postures
    """

    def test_go_to_posture_method(self):
        """
        Test the robustness of the @goToPosture method
        """
        self.assertFalse(
            PostureTest.robot.goToPosture("invalid", 0.5))

    def test_stand_posture(self):
        """
        Test the Stand posture
        """
        self.assertTrue(
            PostureTest.robot.goToPosture("Stand", 0.5))
        self.assertTrue(
            PostureTest.robot.goToPosture("stand", 0.5))

    def test_stand_init_posture(self):
        """
        Test the StandInit posture
        """
        self.assertTrue(
            PostureTest.robot.goToPosture("StandInit", 0.5))
        self.assertTrue(
            PostureTest.robot.goToPosture("standInit", 0.5))

    def test_stand_zero_posture(self):
        """
        Test the StandZero posture
        """
        self.assertTrue(
            PostureTest.robot.goToPosture("StandZero", 0.5))
        self.assertTrue(
            PostureTest.robot.goToPosture("standZero", 0.5))

    def test_crouch_posture(self):
        """
        Test the Crouch posture
        """
        self.assertTrue(
            PostureTest.robot.goToPosture("Crouch", 0.5))
        self.assertTrue(
            PostureTest.robot.goToPosture("crouch", 0.5))


class PepperPostureTest(PostureTest):
    """
    Unittests for Pepper's postures
    """

    @classmethod
    def setUpClass(cls):
        """
        Launches a simulation and spawns the Pepper virtual robot
        """
        PostureTest.simulation = SimulationManager()
        PostureTest.client = PostureTest.simulation.launchSimulation(
            gui=False)

        PostureTest.robot = PostureTest.simulation.spawnPepper(
            PostureTest.client,
            spawn_ground_plane=True)

    @classmethod
    def tearDownClass(cls):
        """
        Stops the simulation
        """
        PostureTest.simulation.stopSimulation(
            PostureTest.client)

    def test_go_to_posture_method(self):
        PostureTest.test_go_to_posture_method(self)

    def test_stand_posture(self):
        PostureTest.test_stand_posture(self)

    def test_stand_zero_posture(self):
        PostureTest.test_stand_zero_posture(self)

    def test_crouch_posture(self):
        PostureTest.test_crouch_posture(self)


class NaoPostureTest(PostureTest):
    """
    Unittests for Nao's postures
    """

    @classmethod
    def setUpClass(cls):
        """
        Launches a simulation and spawns the NAO virtual robot
        """
        PostureTest.simulation = SimulationManager()
        PostureTest.client = PostureTest.simulation.launchSimulation(
            gui=False)

        PostureTest.robot = PostureTest.simulation.spawnNao(
            PostureTest.client,
            spawn_ground_plane=True)

    @classmethod
    def tearDownClass(cls):
        """
        Stops the simulation
        """
        PostureTest.simulation.stopSimulation(
            PostureTest.client)

    def test_go_to_posture_method(self):
        PostureTest.test_go_to_posture_method(self)

    def test_stand_posture(self):
        PostureTest.test_stand_posture(self)

    def test_stand_zero_posture(self):
        PostureTest.test_stand_zero_posture(self)

    def test_crouch_posture(self):
        PostureTest.test_crouch_posture(self)


if __name__ == "__main__":
    unittest.main()
