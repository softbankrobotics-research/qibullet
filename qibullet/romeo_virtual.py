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
        translation = [translation[0], translation[1], translation[2] + 1.0]

        RobotVirtual.loadRobot(
            self,
            translation,
            quaternion,
            physicsClientId=physicsClientId)

        balance_constraint = pybullet.createConstraint(
            parentBodyUniqueId=self.robot_model,
            parentLinkIndex=-1,
            childBodyUniqueId=-1,
            childLinkIndex=-1,
            jointType=pybullet.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=[0, 0, 0],
            parentFrameOrientation=[0, 0, 0, 1],
            childFramePosition=translation,
            childFrameOrientation=quaternion,
            physicsClientId=self.physics_client)

        for link in ["torso", "HeadRoll_link"]:
            pybullet.setCollisionFilterPair(
                self.robot_model,
                self.robot_model,
                self.link_dict["NeckPitch_link"].getIndex(),
                self.link_dict[link].getIndex(),
                0,
                physicsClientId=self.physics_client)

        for side in ["R", "L"]:
            pybullet.setCollisionFilterPair(
                self.robot_model,
                self.robot_model,
                self.link_dict[side + "Eye"].getIndex(),
                self.link_dict["HeadRoll_link"].getIndex(),
                0,
                physicsClientId=self.physics_client)
            pybullet.setCollisionFilterPair(
                self.robot_model,
                self.robot_model,
                self.link_dict[side + "Thigh"].getIndex(),
                self.link_dict["body"].getIndex(),
                0,
                physicsClientId=self.physics_client)
            pybullet.setCollisionFilterPair(
                self.robot_model,
                self.robot_model,
                self.link_dict[side.lower() + "_ankle"].getIndex(),
                self.link_dict[side + "Tibia"].getIndex(),
                0,
                physicsClientId=self.physics_client)
            pybullet.setCollisionFilterPair(
                self.robot_model,
                self.robot_model,
                self.link_dict[side + "ShoulderYaw_link"].getIndex(),
                self.link_dict["torso"].getIndex(),
                0,
                physicsClientId=self.physics_client)
            pybullet.setCollisionFilterPair(
                self.robot_model,
                self.robot_model,
                self.link_dict[side + "WristRoll_link"].getIndex(),
                self.link_dict[side.lower() + "_wrist"].getIndex(),
                0,
                physicsClientId=self.physics_client)

            for link in ["ShoulderYaw_link", "WristYaw_link"]:
                pybullet.setCollisionFilterPair(
                    self.robot_model,
                    self.robot_model,
                    self.link_dict[side + link].getIndex(),
                    self.link_dict[side + "Elbow"].getIndex(),
                    0,
                    physicsClientId=self.physics_client)

            for name, link in self.link_dict.items():
                if side + "finger" in name.lower() or\
                   side + "thumb" in name.lower():
                    pybullet.setCollisionFilterPair(
                        self.robot_model,
                        self.robot_model,
                        self.link_dict[side.lower + "_wrist"].getIndex(),
                        link.getIndex(),
                        0,
                        physicsClientId=self.physics_client)

        pybullet.removeConstraint(
            balance_constraint,
            physicsClientId=self.physics_client)

        for joint_name in list(self.joint_dict):
            if 'RFinger' in joint_name or 'RThumb' in joint_name:
                self.joint_dict[joint_name].setMaxVelocity(
                    self.joint_dict["RHand"].getMaxVelocity())
            elif 'LFinger' in joint_name or 'LThumb' in joint_name:
                self.joint_dict[joint_name].setMaxVelocity(
                    self.joint_dict["LHand"].getMaxVelocity())
