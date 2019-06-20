#!/usr/bin/env python
# coding: utf-8
import unittest
from qibullet import PepperVirtual
from qibullet import SimulationManager


class PepperLaserTest(unittest.TestCase):
    """
    Unittests for Pepper's lasers
    """

    @classmethod
    def setUpClass(cls):
        """
        Launches a simulation and spawns the Pepper virtual robot
        """
        PepperLaserTest.simulation = SimulationManager()
        PepperLaserTest.client = PepperLaserTest.simulation.launchSimulation(
            gui=False)

        PepperLaserTest.robot = PepperLaserTest.simulation.spawnPepper(
            PepperLaserTest.client,
            spawn_ground_plane=True)

    @classmethod
    def tearDownClass(cls):
        """
        Stops the simulation
        """
        PepperLaserTest.simulation.stopSimulation(
            PepperLaserTest.client)

    def test_lasers_data(self):
        """
        Test the retrieval of lasers data
        """
        PepperLaserTest.robot.subscribeLaser()

        self.assertIsInstance(
            PepperLaserTest.robot.getFrontLaserValue(),
            list)
        self.assertIsInstance(
            PepperLaserTest.robot.getRightLaserValue(),
            list)
        self.assertIsInstance(
            PepperLaserTest.robot.getLeftLaserValue(),
            list)

        PepperLaserTest.robot.unsubscribeLaser()

    def test_lasers_display(self):
        """
        Test the lasers display
        """
        PepperLaserTest.robot.subscribeLaser()
        PepperLaserTest.robot.showLaser(True)
        PepperLaserTest.robot.showLaser(False)
        PepperLaserTest.robot.unsubscribeLaser()

    def test_lasers_alive(self):
        """
        Test the isAlive method for the laser extraction process
        """
        self.assertFalse(
            PepperLaserTest.robot.laser_manager.isActive())
        PepperLaserTest.robot.subscribeLaser()
        self.assertTrue(
            PepperLaserTest.robot.laser_manager.isActive())
        PepperLaserTest.robot.unsubscribeLaser()
        self.assertFalse(
            PepperLaserTest.robot.laser_manager.isActive())


if __name__ == "__main__":
    unittest.main()
