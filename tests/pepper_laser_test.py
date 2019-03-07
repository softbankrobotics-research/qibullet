#!/usr/bin/env python
# coding: utf-8
import unittest
from qibullet import PepperVirtual


class PepperLaserTest(unittest.TestCase):
    """
    Unittests for Pepper's lasers
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

    def test_lasers_data(self):
        """
        Test the retrieval of lasers data
        """
        PepperLaserTest.pepper_virtual.subscribeLaser()

        self.assertIsInstance(
            PepperLaserTest.pepper_virtual.getFrontLaserValue(),
            list)
        self.assertIsInstance(
            PepperLaserTest.pepper_virtual.getRightLaserValue(),
            list)
        self.assertIsInstance(
            PepperLaserTest.pepper_virtual.getLeftLaserValue(),
            list)

        PepperLaserTest.pepper_virtual.unsubscribeLaser()

    def test_lasers_display(self):
        """
        Test the lasers display
        """
        PepperLaserTest.pepper_virtual.subscribeLaser()
        PepperLaserTest.pepper_virtual.showLaser(True)
        PepperLaserTest.pepper_virtual.showLaser(False)
        PepperLaserTest.pepper_virtual.unsubscribeLaser()


if __name__ == "__main__":
    unittest.main()
