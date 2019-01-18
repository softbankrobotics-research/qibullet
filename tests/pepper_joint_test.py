#!/usr/bin/env python
# coding: utf-8
import sys
import time
import random
import unittest
import pybullet
from qibullet import PepperVirtual


class PepperJointTest(unittest.TestCase):
    """
    Unittests for the control of Pepper virtual's joints
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

    def test_set_angles(self):
        """
        Test the set @setAngles method
        """
        iterations = 10

        for i in range(iterations):
            angles = list()

            for joint in PepperJointTest.pepper_virtual.joint_dict.values():
                angles.append(random.uniform(
                    joint.getLowerLimit(),
                    joint.getUpperLimit()))
            PepperJointTest.pepper_virtual.setAngles(
                list(PepperJointTest.pepper_virtual.joint_dict),
                angles,
                random.uniform(0.0, 1.0))

            time.sleep(0.2)

    def test_speed_limits(self):
        """
        Test different speed limits for the @setAngles method
        """
        PepperJointTest.pepper_virtual.setAngles("HeadYaw", 1.0, 900)
        PepperJointTest.pepper_virtual.setAngles("HeadYaw", 1.0, -45)

    def test_angle_limits(self):
        """
        Test different angle limits for the @setAngles method
        """
        PepperJointTest.pepper_virtual.setAngles("HeadPitch", -900, 0.5)
        PepperJointTest.pepper_virtual.setAngles("HeadPitch", 900, 0.5)

    def test_get_angles_position(self):
        """
        Test the @getAnglesPosition method
        """
        PepperJointTest.pepper_virtual.getAnglesPosition("HeadYaw")
        PepperJointTest.pepper_virtual.getAnglesPosition(
            PepperJointTest.pepper_virtual.joint_dict.keys())


if __name__ == "__main__":
    unittest.main()
