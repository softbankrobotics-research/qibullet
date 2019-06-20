#!/usr/bin/env python
# coding: utf-8
import sys
import time
import unittest
import pybullet
from qibullet import SimulationManager
from qibullet import NaoVirtual, PepperVirtual


class SelfCollisionTest(unittest.TestCase):
    """
    Unittests for the detection of self collisions
    """

    def test_method_parameters(self):
        """
        Test the @isSelfColliding method parameters (passing list or str,
        giving unauthorized links)
        """
        SelfCollisionTest.robot.isSelfColliding("Head")
        SelfCollisionTest.robot.isSelfColliding("r_wrist")
        SelfCollisionTest.robot.isSelfColliding([
            "RForeArm",
            "LForeArm"])
        SelfCollisionTest.robot.isSelfColliding(
            "non_existing_link")
        SelfCollisionTest.robot.isSelfColliding([
            "non_existing_link_first",
            "non_existing_link_second"])

    def test_wrist_collision(self):
        """
        Test self collisions for the wrists
        """
        self.assertFalse(
            SelfCollisionTest.robot.isSelfColliding("r_wrist"))
        self.assertFalse(
            SelfCollisionTest.robot.isSelfColliding("l_wrist"))

        # Don't need any further tests for NAO
        if isinstance(SelfCollisionTest.robot, NaoVirtual):
            return

        SelfCollisionTest.robot.setAngles(
            "RElbowRoll",
            1.6,
            1.0)
        time.sleep(3)

        self.assertTrue(
            SelfCollisionTest.robot.isSelfColliding("r_wrist"))

        SelfCollisionTest.robot.setAngles(
            "RElbowRoll",
            0.0,
            1.0)
        time.sleep(3)

        SelfCollisionTest.robot.setAngles(
            "LElbowRoll",
            -1.6,
            1.0)
        time.sleep(3)

        self.assertTrue(
            SelfCollisionTest.robot.isSelfColliding("l_wrist"))

        SelfCollisionTest.robot.setAngles(
            "LElbowRoll",
            0.0,
            1.0)
        time.sleep(3)

    def test_forearm_collision(self):
        """
        Test self collisions for the forearms
        """
        self.assertFalse(
            SelfCollisionTest.robot.isSelfColliding("RForeArm"))
        self.assertFalse(
            SelfCollisionTest.robot.isSelfColliding("LForeArm"))

        # Don't need any further tests for NAO
        if isinstance(SelfCollisionTest.robot, NaoVirtual):
            return

        SelfCollisionTest.robot.setAngles(
            ["RShoulderRoll", "RElbowRoll"],
            [-0.3, 1.6],
            1.0)
        time.sleep(3)

        self.assertTrue(
            SelfCollisionTest.robot.isSelfColliding("RForeArm"))

        SelfCollisionTest.robot.setAngles(
            ["RShoulderRoll", "RElbowRoll"],
            [0.0, 0.0],
            1.0)
        time.sleep(3)

        SelfCollisionTest.robot.setAngles(
            ["LShoulderRoll", "LElbowRoll"],
            [0.3, -1.6],
            1.0)
        time.sleep(3)

        self.assertTrue(
            SelfCollisionTest.robot.isSelfColliding("LForeArm"))

        SelfCollisionTest.robot.setAngles(
            ["LShoulderRoll", "LElbowRoll"],
            [0.0, 0.0],
            1.0)
        time.sleep(3)

    def test_head_collision(self):
        """
        Test self collisions for the head
        """
        self.assertFalse(
            SelfCollisionTest.robot.isSelfColliding("Head"))

        # Don't need any further tests for NAO
        if isinstance(SelfCollisionTest.robot, NaoVirtual):
            return

        SelfCollisionTest.robot.setAngles(
            ["RShoulderPitch", "RElbowRoll"],
            [-1.3, 1.3],
            1.0)
        time.sleep(3)

        self.assertTrue(
            SelfCollisionTest.robot.isSelfColliding("Head"))

        SelfCollisionTest.robot.setAngles(
            ["RShoulderPitch", "RElbowRoll"],
            [0.0, 0.0],
            1.0)
        time.sleep(3)


class PepperSelfCollisionTest(SelfCollisionTest):
    """
    Unittests for the detection of Pepper's self collisions
    """

    @classmethod
    def setUpClass(cls):
        """
        Launches a simulation and spawns the Pepper virtual robot
        """
        SelfCollisionTest.simulation = SimulationManager()
        SelfCollisionTest.client = SelfCollisionTest.simulation.launchSimulation(
            gui=False)

        SelfCollisionTest.robot = SelfCollisionTest.simulation.spawnPepper(
            SelfCollisionTest.client,
            spawn_ground_plane=True)

    @classmethod
    def tearDownClass(cls):
        """
        Stops the simulation
        """
        SelfCollisionTest.simulation.stopSimulation(
            SelfCollisionTest.client)


class NaoSelfCollisionTest(SelfCollisionTest):
    """
    Unittests for the detection of Nao's self collisions
    """

    @classmethod
    def setUpClass(cls):
        """
        Launches a simulation and spawns the NAO virtual robot
        """
        SelfCollisionTest.simulation = SimulationManager()
        SelfCollisionTest.client = SelfCollisionTest.simulation.launchSimulation(
            gui=False)

        SelfCollisionTest.robot = SelfCollisionTest.simulation.spawnNao(
            SelfCollisionTest.client,
            spawn_ground_plane=True)

    @classmethod
    def tearDownClass(cls):
        """
        Stops the simulation
        """
        SelfCollisionTest.simulation.stopSimulation(
            SelfCollisionTest.client)


if __name__ == "__main__":
    unittest.main()
