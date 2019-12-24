#!/usr/bin/env python
# coding: utf-8
import unittest
import pybullet
from qibullet import SimulationManager
from qibullet.robot_virtual import RobotVirtual


class VirtualRobotTest(unittest.TestCase):
    """
    Unittests for the RobotVirtual class functionalities (will use a Pepper
    robot for the test, PepperVirtual inherits from RobotVirtual)
    """

    @classmethod
    def setUpClass(cls):
        """
        Launches a simulation and spawns a Pepper virtual robot
        """
        VirtualRobotTest.simulation = SimulationManager()
        VirtualRobotTest.client = VirtualRobotTest.simulation.launchSimulation(
            gui=False)

        VirtualRobotTest.robot = DummyVirtual()

    @classmethod
    def tearDownClass(cls):
        """
        Stops the simulation
        """
        VirtualRobotTest.simulation.stopSimulation(
            VirtualRobotTest.client)

    def test_failing_robot_model(self):
        """
        Assert that a pybullet.error is raised if loadURDF cannot load the
        robot's description
        """
        with self.assertRaises(pybullet.error):
            VirtualRobotTest.robot.loadRobot(
                physicsClientId=VirtualRobotTest.client)

    def test_get_client_id(self):
        """
        Test the physics client id getter
        """
        self.assertEqual(
            VirtualRobotTest.client,
            VirtualRobotTest.robot.getPhysicsClientId())

    def test_get_camera_frame_no_active_camera(self):
        """
        Assert that getCameraFrame raises a pybullet.error when there is no
        active camera
        """
        with self.assertRaises(pybullet.error):
            VirtualRobotTest.robot.getCameraFrame()

    def test_get_camera_resolution_no_active_camera(self):
        """
        Assert that getCameraResolution raises a pybullet.error when there is
        no active camera
        """
        with self.assertRaises(pybullet.error):
            VirtualRobotTest.robot.getCameraFrame()


class DummyVirtual(RobotVirtual):
    """
    Dummy virtual robot class, used to test the RobotVirtual class
    """
    URDF_PATH = "Incorrect path"
    ID_CAMERA_DUMMY = -3

    def __init__(self):
        """
        Constructor
        """
        RobotVirtual.__init__(self, DummyVirtual.URDF_PATH)

    def loadRobot(self, physicsClientId=0):
        """
        Dummy overload to test the loadRobot method
        """
        RobotVirtual.loadRobot(
            self,
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            physicsClientId=physicsClientId)


if __name__ == "__main__":
    unittest.main()
