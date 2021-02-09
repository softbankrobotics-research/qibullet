#!/usr/bin/env python
# coding: utf-8
import unittest
import pybullet
from qibullet.fsr import Fsr
from qibullet.fsr import FsrHandler
from qibullet.fsr import NaoFsr
from qibullet import SimulationManager
from qibullet import NaoVirtual
from qibullet.robot_virtual import RobotVirtual


class FsrTest(unittest.TestCase):
    """
    Unittests for a Fsr sensor (Fsr object)
    """

    def test_fsr_names(self):
        """
        Test the names of the fsrs (for instance the names defined in NaoFsr,
        check the test daughter classes for more information)
        """
        names = list()

        for group in FsrTest.fsr_groups:
            names.extend(group)

        for name in names:
            self.assertIn(name, FsrTest.fsr_names)

    def test_get_fsr_value(self):
        """
        Test the @getFsrValue method
        """
        for name in FsrTest.fsr_names:
            self.assertIsInstance(FsrTest.robot.getFsrValue(name), float)

        # Pass an unexisting fsr name
        with self.assertRaises(pybullet.error):
            FsrTest.robot.getFsrValue("Name that does not exist")

    def test_get_fsr_values(self):
        """
        Test the @getFsrValues method
        """
        # Test for non-empty and empty lists of FSR names
        groups = FsrTest.fsr_groups
        groups.extend([])

        for group in FsrTest.fsr_groups:
            values = FsrTest.robot.getFsrValues(group)
            self.assertIsInstance(values, list)
            self.assertEqual(len(values), len(group))

        # Pass a list of fsr names containing an unexisting FSR
        with self.assertRaises(pybullet.error):
            FsrTest.robot.getFsrValues(["does not exist"])

    def test_get_total_fsr_values(self):
        """
        Test the @getTotalFsrValues method
        """
        # Test for non-empty and empty lists of FSR names
        groups = FsrTest.fsr_groups
        groups.extend([])

        for group in FsrTest.fsr_groups:
            values = FsrTest.robot.getTotalFsrValues(group)
            self.assertIsInstance(values, float)

        # Pass a list of fsr names containing an unexisting FSR
        with self.assertRaises(pybullet.error):
            FsrTest.robot.getTotalFsrValues(["does not exist"])

    def test_get_fsr_handler(self):
        """
        Test the @getFsrHandler method
        """
        self.assertIsInstance(FsrTest.robot.getFsrHandler(), FsrHandler)


class NaoFsrTest(FsrTest):
    """
    Unittests for Nao virtual's FSRs
    """

    @classmethod
    def setUpClass(cls):
        """
        Launches a simulation and spawns the NAO virtual robot
        """
        FsrTest.simulation = SimulationManager()
        FsrTest.client = FsrTest.simulation.launchSimulation(
            gui=False)

        FsrTest.fsr_names = NaoFsr.RFOOT + NaoFsr.LFOOT
        FsrTest.fsr_groups = [NaoFsr.RFOOT, NaoFsr.LFOOT]

        FsrTest.robot = FsrTest.simulation.spawnNao(
            FsrTest.client,
            spawn_ground_plane=True)

    @classmethod
    def tearDownClass(cls):
        """
        Stops the simulation
        """
        FsrTest.simulation.stopSimulation(
            FsrTest.client)


class DummyFsrTest(unittest.TestCase):
    """
    Unittests for the behaviour of the Fsr related methods of RobotVirtual,
    when the FSR handler is None (default value)
    """

    @classmethod
    def setUpClass(cls):
        """
        Launches a simulation and spawns the NAO virtual robot
        """
        DummyFsrTest.simulation = SimulationManager()
        DummyFsrTest.client = DummyFsrTest.simulation.launchSimulation(
            gui=False)

        DummyFsrTest.robot = DummyRobot()

    @classmethod
    def tearDownClass(cls):
        """
        Stops the simulation
        """
        DummyFsrTest.simulation.stopSimulation(
            DummyFsrTest.client)

    def test_fsr_handler_none(self):
        """
        Test the behaviour of the FSR related methods of RobotVirtual when the
        FSR handler of the robot doesn't exist (set to None)
        """
        with self.assertRaises(pybullet.error):
            DummyFsrTest.robot.getFsrValue("dummy_name")

        with self.assertRaises(pybullet.error):
            DummyFsrTest.robot.getFsrValues(["dummy_name"])

        with self.assertRaises(pybullet.error):
            DummyFsrTest.robot.getTotalFsrValues(["dummy_name"])

        self.assertIsNone(DummyFsrTest.robot.getFsrHandler())


class DummyRobot(RobotVirtual):
    """
    Dummy robot class
    """
    def __init__(self):
        """
        Constructor
        """
        RobotVirtual.__init__(self, "wrong_description_filepath")


if __name__ == "__main__":
    unittest.main()
