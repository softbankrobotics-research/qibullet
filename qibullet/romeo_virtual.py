#!/usr/bin/env python
# coding: utf-8

import os
import pybullet
from qibullet.camera import *
from qibullet.robot_virtual import RobotVirtual


class RomeoVirtual(RobotVirtual):
    """
    Class representing the virtual instance of the Romeo robot
    """
    ID_CAMERA_TOP = 0
    ID_CAMERA_BOTTOM = 1
    FRAME_WORLD = 1
    FRAME_ROBOT = 2
    URDF_PATH = "robot_data/romeo_H37/romeo.urdf"

    def __init__(self):
        """
        Constructor
        """
        RobotVirtual.__init__(self, RomeoVirtual.URDF_PATH)
        self.camera_top = None
        self.camera_bottom = None

    def loadRobot(self, translation, quaternion, physicsClientId=0):
        """
        Overloads @loadRobot from the @RobotVirtual class, loads the robot into
        the simulated instance. This method also updates the max velocity of
        the robot's fingers, adds self collision filters to the model and
        defines the cameras of the model.

        Parameters:
            translation - List containing 3 elements, the translation [x, y, z]
            of the robot in the WORLD frame
            quaternion - List containing 4 elements, the quaternion
            [x, y, z, q] of the robot in the WORLD frame
            physicsClientId - The id of the simulated instance in which the
            robot is supposed to be loaded

        Returns:
            boolean - True if the method ran correctly, False otherwise
        """
        pybullet.setAdditionalSearchPath(
            os.path.dirname(os.path.realpath(__file__)),
            physicsClientId=physicsClientId)

        # Add 0.50 meters on the z component, avoing to spawn NAO in the ground
        translation = [translation[0], translation[1], translation[2] + 0.50]

        RobotVirtual.loadRobot(
            self,
            translation,
            quaternion,
            physicsClientId=physicsClientId)
