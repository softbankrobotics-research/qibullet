#!/usr/bin/env python
# coding: utf-8
import unittest
from qibullet import PepperVirtual


class PepperPostureTest(unittest.TestCase):
    """
    Unittests for Pepper's postures
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

    def test_go_to_posture_method(self):
        """
        Test the robustness of the @goToPosture method
        """
        self.assertFalse(
            PepperPostureTest.pepper_virtual.goToPosture("invalid", 0.5))

    def test_stand_posture(self):
        """
        Test the Stand / StandInit posture
        """
        self.assertTrue(
            PepperPostureTest.pepper_virtual.goToPosture("Stand", 0.5))
        self.assertTrue(
            PepperPostureTest.pepper_virtual.goToPosture("StandInit", 0.5))
        self.assertTrue(
            PepperPostureTest.pepper_virtual.goToPosture("stand", 0.5))
        self.assertTrue(
            PepperPostureTest.pepper_virtual.goToPosture("standInit", 0.5))

    def test_stand_zero_posture(self):
        """
        Test the StandZero posture
        """
        self.assertTrue(
            PepperPostureTest.pepper_virtual.goToPosture("StandZero", 0.5))
        self.assertTrue(
            PepperPostureTest.pepper_virtual.goToPosture("standZero", 0.5))

    def test_crouch_posture(self):
        """
        Test the Crouch posture
        """
        self.assertTrue(
            PepperPostureTest.pepper_virtual.goToPosture("Crouch", 0.5))
        self.assertTrue(
            PepperPostureTest.pepper_virtual.goToPosture("crouch", 0.5))


if __name__ == "__main__":
    unittest.main()
