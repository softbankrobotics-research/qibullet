#!/usr/bin/env python
# coding: utf-8

import os
import pybullet
from qibullet.camera import *
from qibullet.robot_posture import RomeoPosture
from qibullet.robot_virtual import RobotVirtual


class RomeoVirtual(RobotVirtual):
    """
    Class representing the virtual instance of the Romeo robot
    """
    ID_CAMERA_RIGHT = 0
    ID_CAMERA_LEFT = 1
    ID_CAMERA_DEPTH = 2
    FRAME_WORLD = 1
    FRAME_ROBOT = 2
    URDF_PATH = "robot_data/romeo_H37/romeo.urdf"
    P_STAND = RomeoPosture("Stand")
    P_STAND_INIT = RomeoPosture("StandInit")
    P_STAND_ZERO = RomeoPosture("StandZero")
    P_CROUCH = RomeoPosture("Crouch")

    def __init__(self):
        """
        Constructor
        """
        RobotVirtual.__init__(self, RomeoVirtual.URDF_PATH)
        self.camera_right = None
        self.camera_left = None
        self.camera_depth = None

    def loadRobot(self, translation, quaternion, physicsClientId=0):
        """
        Overloads @loadRobot from the @RobotVirtual class, loads the robot into
        the simulated instance. This method also updates the max velocity of
        the robot's fingers, adds self collision filters to the model and
        defines the cameras of the model in the camera_dict.

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
        translation = [translation[0], translation[1], translation[2] + 1.05]

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

        self.goToPosture("Stand", 1.0)

        pybullet.setCollisionFilterPair(
            self.robot_model,
            self.robot_model,
            self.link_dict["REye"].getIndex(),
            self.link_dict["LEye"].getIndex(),
            0,
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

        for joint in self.joint_dict.values():
            pybullet.resetJointState(
                self.robot_model,
                joint.getIndex(),
                0.0)

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

        camera_right = CameraRgb(
            self.robot_model,
            RomeoVirtual.ID_CAMERA_RIGHT,
            self.link_dict["CameraRightEye_optical_frame"],
            hfov=60.9,
            vfov=47.6,
            physicsClientId=self.physics_client)

        camera_left = CameraRgb(
            self.robot_model,
            RomeoVirtual.ID_CAMERA_LEFT,
            self.link_dict["CameraLeftEye_optical_frame"],
            hfov=60.9,
            vfov=47.6,
            physicsClientId=self.physics_client)

        camera_depth = CameraDepth(
            self.robot_model,
            RomeoVirtual.ID_CAMERA_DEPTH,
            self.link_dict["CameraDepth_optical_frame"],
            hfov=58.0,
            vfov=45.0,
            physicsClientId=self.physics_client)

        self.camera_dict = {
            RomeoVirtual.ID_CAMERA_RIGHT: camera_right,
            RomeoVirtual.ID_CAMERA_LEFT: camera_left,
            RomeoVirtual.ID_CAMERA_DEPTH: camera_depth}

    def setAngles(self, joint_names, joint_values, percentage_speed):
        """
        Overloads @setAngles from the @RobotVirtual class. Handles the finger
        mimic behaviour.

        Parameters:
            joint_names - List of string (or string if only one joint)
            containing the name of the joints to be controlled
            joint_values - List of values (or value if only one joint)
            corresponding to the angles in radians to be applied
            percentage_speed - Percentage (or percentages) of the max speed to
            be used for the movement
        """
        try:
            if type(joint_names) is str:
                assert type(joint_values) is int or type(joint_values) is float
                names = [joint_names]
                values = [joint_values]
            else:
                assert type(joint_names) is type(joint_values) is list
                names = list(joint_names)
                values = list(joint_values)

            if isinstance(percentage_speed, list):
                speeds = list(percentage_speed)
            else:
                speeds = [percentage_speed]*len(names)

        except AssertionError:
            raise pybullet.error("Error in the parameters given to the\
                setAngles method")

        for hand in ["RHand", "LHand"]:
            for i in range(names.count(hand)):
                index = names.index(hand)
                value = values[index]
                speed = speeds[index]
                names.pop(index)
                values.pop(index)
                speeds.pop(index)
                finger_names, finger_values = self._mimicHand(hand, value)
                names.extend(finger_names)
                values.extend(finger_values)
                speeds.extend([speed]*len(finger_names))

        RobotVirtual.setAngles(
            self,
            names,
            values,
            speeds)

    def getAnglesPosition(self, joint_names):
        """
        Overloads @getAnglesPosition from the @RobotVirtual class. Handles the
        finger mimicked position for the hands (when getting the position of
        RHand or LHand, will return the hand's opening percentage).

        Parameters:
            joint_names - List of string (or string if only one joint)
            containing the name of the joints

        Returns:
            joint_positions - List of float (or float if only one joint)
            containing the joint's positions in radians
        """
        if type(joint_names) is str:
            names = [joint_names]
        else:
            names = list(joint_names)

        joint_positions = RobotVirtual.getAnglesPosition(self, names)

        for hand, finger in zip(
                ["RHand", "LHand"],
                ["RFinger11", "LFinger11"]):
            for i in range(names.count(hand)):
                index = names.index(hand)
                joint_positions[index] =\
                    RobotVirtual.getAnglesPosition(self, [finger]).pop() /\
                    (1/(self.joint_dict[finger].getUpperLimit() -
                        self.joint_dict[finger].getLowerLimit())) +\
                    self.joint_dict[finger].getLowerLimit()

        if len(joint_positions) == 1:
            return joint_positions.pop()
        else:
            return joint_positions

    def getAnglesVelocity(self, joint_names):
        """
        Overloads @getAnglesVelocity from the @RobotVirtual class. The method
        won't return the velocity of RHand and LHand joints.

        Parameters:
            joint_names - List of string (or string if only one joint)
            containing the name of the joints

        Returns:
            joint_velocities - List of float (or float if only one joint)
            containing the joint's velocities in rad/s
        """
        if type(joint_names) is str:
            names = [joint_names]
        else:
            names = list(joint_names)

        joint_velocities = RobotVirtual.getAnglesVelocity(self, names)

        if len(joint_velocities) == 1:
            return joint_velocities.pop()
        else:
            return joint_velocities

    def goToPosture(self, posture_name, percentage_speed):
        """
        Position the virtual robot into a particular posture. The different
        available postures are NaoPosture objects.

        Parameters:
            posture_name - String containing the name of the posture. The
            posture name is not case-sensitive
            percentage_speed - Percentage of the max speed to be used for the
            movement

        Returns:
            Boolean - True if the posture can be applied, False otherwise
        """
        posture_list = [
            RomeoVirtual.P_STAND,
            RomeoVirtual.P_STAND_INIT,
            RomeoVirtual.P_STAND_ZERO,
            RomeoVirtual.P_CROUCH]

        for posture in posture_list:
            if posture.isPostureName(posture_name):
                self.setAngles(
                    posture.getPostureJointNames(),
                    posture.getPostureJointValues(),
                    percentage_speed)

                return True

        return False

    def _mimicHand(
            self,
            hand,
            value,
            multiplier=0.988205,
            thumb_multiplier=1.74533,
            offset=0.0):
        """
        Used to propagate a joint value on the fingers attached to the hand.
        The formula used to mimic a joint is the following:

        finger_value = (hand_value * multiplier) + offset

        Parameters:
            hand - String, RHand or LHand
            value - The joint value to be propagated
            multiplier - The multiplier coefficient for the fingers and the
            sections 2 and 3 of the thumbs (0.988205 by default)
            thumb_multiplier - The multiplier coefficient for the first section
            of the thumb (1.74533 by default)
            offset - The offset coefficient (0.0 by default)

        Returns:
            finger_names - Names of the finger to be controlled
            finger_values - Values of the fingers to be controlled
        """
        finger_names = list()
        finger_values = list()

        for joint_name in self.joint_dict.keys():
            if hand[0] + "Thumb1" in joint_name:
                finger_names.append(joint_name)
                finger_values.append((value * thumb_multiplier) + offset)
            elif (hand[0] + "Finger") in joint_name or\
                 (hand[0] + "Thumb") in joint_name:
                finger_names.append(joint_name)
                finger_values.append((value * multiplier) + offset)

        return finger_names, finger_values
