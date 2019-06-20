#!/usr/bin/env python
# coding: utf-8
import sys
import time
import random
import unittest
import pybullet
from qibullet import SimulationManager
from qibullet import NaoVirtual, PepperVirtual


class JointTest(unittest.TestCase):
    """
    Unittests for the virtual joints (virtual class don't use directly)
    """

    def test_set_angles(self):
        """
        Test the set @setAngles method
        """
        iterations = 10

        JointTest.robot.setAngles(
            ["HeadYaw", "HeadPitch"],
            [0.5, 0.3],
            0.1)

        time.sleep(0.2)

        JointTest.robot.setAngles(
            ["HeadYaw", "HeadPitch"],
            [0.1, 0.0],
            [0.8, 0.6])

        time.sleep(0.2)

        for i in range(iterations):
            angles = list()

            for joint in JointTest.robot.joint_dict.values():
                angles.append(random.uniform(
                    joint.getLowerLimit(),
                    joint.getUpperLimit()))
            JointTest.robot.setAngles(
                list(JointTest.robot.joint_dict),
                angles,
                random.uniform(0.0, 1.0))

            time.sleep(0.2)

    def test_speed_limits(self):
        """
        Test different speed limits for the @setAngles method
        """
        try:
            JointTest.robot.setAngles("HeadYaw", 1.0, 900)
        except pybullet.error:
            pass

        try:
            JointTest.robot.setAngles("HeadYaw", 1.0, -45)
        except pybullet.error:
            pass

    def test_angle_limits(self):
        """
        Test different angle limits for the @setAngles method
        """
        JointTest.robot.setAngles("HeadPitch", -900, 0.5)
        JointTest.robot.setAngles("HeadPitch", 900, 0.5)

    def test_get_angles_position(self):
        """
        Test the @getAnglesPosition method
        """
        JointTest.robot.getAnglesPosition("HeadYaw")
        JointTest.robot.getAnglesPosition(
            JointTest.robot.joint_dict.keys())


class PepperJointTest(JointTest):
    """
    Unittests for the control of Pepper virtual's joints
    """

    @classmethod
    def setUpClass(cls):
        """
        Launches a simulation and spawns the Pepper virtual robot
        """
        JointTest.simulation = SimulationManager()
        JointTest.client = JointTest.simulation.launchSimulation(
            gui=False)

        JointTest.robot = JointTest.simulation.spawnPepper(
            JointTest.client,
            spawn_ground_plane=True)

    @classmethod
    def tearDownClass(cls):
        """
        Stops the simulation
        """
        JointTest.simulation.stopSimulation(
            JointTest.client)

    def test_set_angles(self):
        JointTest.test_set_angles(self)

    def test_speed_limits(self):
        JointTest.test_speed_limits(self)

    def test_angle_limits(self):
        JointTest.test_angle_limits(self)

    def test_get_angles_position(self):
        JointTest.test_get_angles_position(self)


class NaoJointTest(JointTest):
    """
    Unittests for the control of Nao virtual's joints
    """

    @classmethod
    def setUpClass(cls):
        """
        Launches a simulation and spawns the NAO virtual robot
        """
        JointTest.simulation = SimulationManager()
        JointTest.client = JointTest.simulation.launchSimulation(
            gui=False)

        JointTest.robot = JointTest.simulation.spawnNao(
            JointTest.client,
            spawn_ground_plane=True)

    @classmethod
    def tearDownClass(cls):
        """
        Stops the simulation
        """
        JointTest.simulation.stopSimulation(
            JointTest.client)

    def test_set_angles(self):
        JointTest.test_set_angles(self)

    def test_speed_limits(self):
        JointTest.test_speed_limits(self)

    def test_angle_limits(self):
        JointTest.test_angle_limits(self)

    def test_get_angles_position(self):
        JointTest.test_get_angles_position(self)


if __name__ == "__main__":
    unittest.main()
