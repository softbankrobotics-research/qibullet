#!/usr/bin/env python
# coding: utf-8
import sys
import time
import random
import unittest
import pybullet
from qibullet import SimulationManager
from qibullet import NaoVirtual, PepperVirtual, RomeoVirtual


class JointTest(unittest.TestCase):
    """
    Unittests for the virtual joints (virtual class don't use directly)
    """

    def test_joints_characteristics(self):
        """
        Test the behaviour of the Joint class with the robot's characteristics
        """
        for key, value in JointTest.robot.joint_dict.items():
            index = value.index
            name = value.name
            lower_limit = value.lower_limit
            upper_limit = value.upper_limit
            max_effort = value.max_effort
            max_velocity = value.max_velocity

            self.assertEqual(key, value.getName())
            self.assertEqual(index, value.getIndex())
            self.assertEqual(name, value.getName())
            self.assertEqual(lower_limit, value.getLowerLimit())
            self.assertEqual(upper_limit, value.getUpperLimit())
            self.assertEqual(max_effort, value.getMaxEffort())
            self.assertEqual(max_velocity, value.getMaxVelocity())

    def test_set_angles(self):
        """
        Test the set @setAngles method
        """
        iterations = 10

        if isinstance(JointTest.robot, RomeoVirtual):
            angle_names = ["HeadRoll", "HeadPitch"]
        else:
            angle_names = ["HeadYaw", "HeadPitch"]

        JointTest.robot.setAngles(
            angle_names,
            [0.3, 0.3],
            0.1)

        time.sleep(0.2)

        JointTest.robot.setAngles(
            angle_names,
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
        with self.assertRaises(pybullet.error):
            if isinstance(JointTest.robot, RomeoVirtual):
                JointTest.robot.setAngles("HeadRoll", 0.4, 900)
            else:
                JointTest.robot.setAngles("HeadYaw", 1.0, 900)

        with self.assertRaises(pybullet.error):
            if isinstance(JointTest.robot, RomeoVirtual):
                JointTest.robot.setAngles("HeadRoll", 0.4, -45)
            else:
                JointTest.robot.setAngles("HeadYaw", 1.0, -45)

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
        self.assertTrue(isinstance(
            JointTest.robot.getAnglesPosition("HeadPitch"),
            float))

        positions = JointTest.robot.getAnglesPosition(
            JointTest.robot.joint_dict.keys())

        self.assertTrue(isinstance(positions, list))

    def test_get_angles_velocity(self):
        """
        Test the @getAnglesVelocity method
        """
        self.assertTrue(isinstance(
            JointTest.robot.getAnglesVelocity("HeadPitch"),
            float))

        velocities = JointTest.robot.getAnglesVelocity(
            JointTest.robot.joint_dict.keys())

        self.assertTrue(isinstance(velocities, list))


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

    def test_joints_characteristics(self):
        JointTest.test_joints_characteristics(self)

    def test_set_angles(self):
        JointTest.test_set_angles(self)

    def test_speed_limits(self):
        JointTest.test_speed_limits(self)

    def test_angle_limits(self):
        JointTest.test_angle_limits(self)

    def test_get_angles_position(self):
        JointTest.test_get_angles_position(self)

    def test_get_angles_velocity(self):
        JointTest.test_get_angles_velocity(self)


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

    def test_joints_characteristics(self):
        JointTest.test_joints_characteristics(self)

    def test_set_angles(self):
        JointTest.test_set_angles(self)

    def test_speed_limits(self):
        JointTest.test_speed_limits(self)

    def test_angle_limits(self):
        JointTest.test_angle_limits(self)

    def test_get_angles_position(self):
        JointTest.test_get_angles_position(self)

    def test_get_angles_velocity(self):
        JointTest.test_get_angles_velocity(self)


class RomeoJointTest(JointTest):
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

        JointTest.robot = JointTest.simulation.spawnRomeo(
            JointTest.client,
            spawn_ground_plane=True)

    @classmethod
    def tearDownClass(cls):
        """
        Stops the simulation
        """
        JointTest.simulation.stopSimulation(
            JointTest.client)

    def test_joints_characteristics(self):
        JointTest.test_joints_characteristics(self)

    def test_set_angles(self):
        JointTest.test_set_angles(self)

    def test_speed_limits(self):
        JointTest.test_speed_limits(self)

    def test_angle_limits(self):
        JointTest.test_angle_limits(self)

    def test_get_angles_position(self):
        JointTest.test_get_angles_position(self)

    def test_get_angles_velocity(self):
        JointTest.test_get_angles_velocity(self)


if __name__ == "__main__":
    unittest.main()
