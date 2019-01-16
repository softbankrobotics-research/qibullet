#!/usr/bin/env python
# coding: utf-8
import sys
import time
import unittest
import pybullet
from qibullet import PepperVirtual


class PepperSelfCollisionTest(unittest.TestCase):
    """
    Unittests for the detection of PepperVirtual's self collisions
    """

    @classmethod
    def setUpClass(cls):
        """
        Launches a simulation and spawns the Pepper virtual robot
        """
        cls.pepper_virtual = PepperVirtual()
        cls.pepper_virtual.loadRobot(
            [0, 0, 0],
            [0, 0, 0, 1])

    def test_method_parameters(self):
        """
        Test the @isSelfColliding method parameters (passing list or str,
        giving unauthorized links)
        """
        PepperSelfCollisionTest.pepper_virtual.isSelfColliding("Head")
        PepperSelfCollisionTest.pepper_virtual.isSelfColliding("r_wrist")
        PepperSelfCollisionTest.pepper_virtual.isSelfColliding([
            "RForeArm",
            "LForeArm"])
        PepperSelfCollisionTest.pepper_virtual.isSelfColliding(
            "non_existing_link")
        PepperSelfCollisionTest.pepper_virtual.isSelfColliding([
            "non_existing_link_first",
            "non_existing_link_second"])

    def test_wrist_collision(self):
        """
        Test self collisions for the wrists
        """
        self.assertFalse(
            PepperSelfCollisionTest.pepper_virtual.isSelfColliding("r_wrist"))

        PepperSelfCollisionTest.pepper_virtual.setAngles(
            "RElbowRoll",
            1.6,
            1.0)
        time.sleep(0.5)

        self.assertTrue(
            PepperSelfCollisionTest.pepper_virtual.isSelfColliding("r_wrist"))

        PepperSelfCollisionTest.pepper_virtual.setAngles(
            "RElbowRoll",
            0.0,
            1.0)
        time.sleep(0.5)

        self.assertFalse(
            PepperSelfCollisionTest.pepper_virtual.isSelfColliding("l_wrist"))

        PepperSelfCollisionTest.pepper_virtual.setAngles(
            "LElbowRoll",
            -1.6,
            1.0)
        time.sleep(0.5)

        self.assertTrue(
            PepperSelfCollisionTest.pepper_virtual.isSelfColliding("l_wrist"))

        PepperSelfCollisionTest.pepper_virtual.setAngles(
            ["RElbowRoll", "LElbowRoll"],
            [0.7, -0.7],
            1.0)
        time.sleep(0.5)

        self.assertTrue(
            PepperSelfCollisionTest.pepper_virtual.isSelfColliding("r_wrist"))
        self.assertTrue(
            PepperSelfCollisionTest.pepper_virtual.isSelfColliding("l_wrist"))

        PepperSelfCollisionTest.pepper_virtual.setAngles(
            ["RElbowRoll", "LElbowRoll"],
            [0.0, 0.0],
            1.0)
        time.sleep(0.5)

    def test_forearm_collision(self):
        """
        Test self collisions for the forearms
        """
        self.assertFalse(
            PepperSelfCollisionTest.pepper_virtual.isSelfColliding("RForeArm"))
        self.assertFalse(
            PepperSelfCollisionTest.pepper_virtual.isSelfColliding("LForeArm"))

        PepperSelfCollisionTest.pepper_virtual.setAngles(
            ["RShoulderRoll", "RElbowRoll"],
            [-0.3, 1.6],
            1.0)
        time.sleep(0.5)

        self.assertTrue(
            PepperSelfCollisionTest.pepper_virtual.isSelfColliding("RForeArm"))

        PepperSelfCollisionTest.pepper_virtual.setAngles(
            ["RShoulderRoll", "RElbowRoll"],
            [0.0, 0.0],
            1.0)
        time.sleep(0.5)

        PepperSelfCollisionTest.pepper_virtual.setAngles(
            ["LShoulderRoll", "LElbowRoll"],
            [0.3, -1.6],
            1.0)
        time.sleep(0.5)

        self.assertTrue(
            PepperSelfCollisionTest.pepper_virtual.isSelfColliding("LForeArm"))

        PepperSelfCollisionTest.pepper_virtual.setAngles(
            ["LShoulderRoll", "LElbowRoll"],
            [0.0, 0.0],
            1.0)
        time.sleep(0.5)

    def test_head_collision(self):
        """
        Test self collisions for the head
        """
        self.assertFalse(
            PepperSelfCollisionTest.pepper_virtual.isSelfColliding("Head"))

        PepperSelfCollisionTest.pepper_virtual.setAngles(
            ["RShoulderPitch", "RElbowRoll"],
            [-1.3, 1.3],
            1.0)
        time.sleep(0.5)

        self.assertTrue(
            PepperSelfCollisionTest.pepper_virtual.isSelfColliding("Head"))

        PepperSelfCollisionTest.pepper_virtual.setAngles(
            ["RShoulderPitch", "RElbowRoll"],
            [0.0, 0.0],
            1.0)
        time.sleep(0.5)


if __name__ == "__main__":
    unittest.main()
