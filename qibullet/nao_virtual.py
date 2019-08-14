#!/usr/bin/env python
# coding: utf-8

import os
import pybullet
from qibullet.camera import *
from qibullet.base_controller import *
from qibullet.robot_posture import NaoPosture
from qibullet.robot_virtual import RobotVirtual


class NaoVirtual(RobotVirtual):
    """
    Class representing the virtual instance of the NAO robot
    """
    ID_CAMERA_TOP = 0
    ID_CAMERA_BOTTOM = 1
    FRAME_WORLD = 1
    FRAME_ROBOT = 2
    URDF_PATH = "robot_data/nao_V40/nao.urdf"
    P_STAND = NaoPosture("Stand")
    P_STAND_INIT = NaoPosture("StandInit")
    P_STAND_ZERO = NaoPosture("StandZero")
    P_CROUCH = NaoPosture("Crouch")
    P_SIT = NaoPosture("Sit")
    P_SIT_RELAX = NaoPosture("SitRelax")
    P_LYING_BELLY = NaoPosture("LyingBelly")
    P_LYING_BACK = NaoPosture("LyingBack")

    def __init__(self):
        """
        Constructor
        """
        RobotVirtual.__init__(self, NaoVirtual.URDF_PATH)
        self.camera_top = None
        self.camera_bottom = None
        # TODO: add constraints and speeds

    def loadRobot(self, translation, quaternion, physicsClientId=0):
        """
        Overloads @loadRobot from the @RobotVirtual class. Update max velocity
        for the fingers and thumbs, based on the hand joints. Add self
        collision exceptions. Add the cameras.
        """
        pybullet.setAdditionalSearchPath(
            os.path.dirname(os.path.realpath(__file__)),
            physicsClientId=physicsClientId)

        # Add 0.36 meters on the z component, avoing to spawn NAO in the ground
        translation = [translation[0], translation[1], translation[2] + 0.36]

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
            self.link_dict["torso"].getIndex(),
            self.link_dict["Head"].getIndex(),
            0,
            physicsClientId=self.physics_client)

        for side in ["R", "L"]:
            pybullet.setCollisionFilterPair(
                self.robot_model,
                self.robot_model,
                self.link_dict[side + "Thigh"].getIndex(),
                self.link_dict[side + "Hip"].getIndex(),
                0,
                physicsClientId=self.physics_client)
            pybullet.setCollisionFilterPair(
                self.robot_model,
                self.robot_model,
                self.link_dict[side + "Bicep"].getIndex(),
                self.link_dict[side + "ForeArm"].getIndex(),
                0,
                physicsClientId=self.physics_client)
            pybullet.setCollisionFilterPair(
                self.robot_model,
                self.robot_model,
                self.link_dict[side + "Pelvis"].getIndex(),
                self.link_dict[side + "Thigh"].getIndex(),
                0,
                physicsClientId=self.physics_client)
            pybullet.setCollisionFilterPair(
                self.robot_model,
                self.robot_model,
                self.link_dict[side + "Tibia"].getIndex(),
                self.link_dict[side.lower() + "_ankle"].getIndex(),
                0,
                physicsClientId=self.physics_client)
            pybullet.setCollisionFilterPair(
                self.robot_model,
                self.robot_model,
                self.link_dict[side + "Finger11_link"].getIndex(),
                self.link_dict[side + "Finger13_link"].getIndex(),
                0,
                physicsClientId=self.physics_client)
            pybullet.setCollisionFilterPair(
                self.robot_model,
                self.robot_model,
                self.link_dict[side + "Finger21_link"].getIndex(),
                self.link_dict[side + "Finger23_link"].getIndex(),
                0,
                physicsClientId=self.physics_client)

        for base_link in ["RThigh", "LThigh", "RBicep", "LBicep"]:
            pybullet.setCollisionFilterPair(
                self.robot_model,
                self.robot_model,
                self.link_dict["torso"].getIndex(),
                self.link_dict[base_link].getIndex(),
                0,
                physicsClientId=self.physics_client)

        for name, link in self.link_dict.items():
            for wrist in ["r_wrist", "l_wrist"]:
                if wrist[0] + "finger" in name.lower() or\
                   wrist[0] + "thumb" in name.lower():
                    pybullet.setCollisionFilterPair(
                        self.robot_model,
                        self.robot_model,
                        self.link_dict[wrist].getIndex(),
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

        self.camera_top = CameraRgb(
            self.robot_model,
            self.link_dict["CameraTop_optical_frame"],
            hfov=60.9,
            vfov=47.6,
            physicsClientId=self.physics_client)

        self.camera_bottom = CameraRgb(
            self.robot_model,
            self.link_dict["CameraBottom_optical_frame"],
            hfov=60.9,
            vfov=47.6,
            physicsClientId=self.physics_client)

        # eventual constraints and lasers

    # TODO: implement a moveTo
    # TODO: implement a move

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
            NaoVirtual.P_STAND,
            NaoVirtual.P_STAND_INIT,
            NaoVirtual.P_STAND_ZERO,
            NaoVirtual.P_CROUCH,
            NaoVirtual.P_SIT,
            NaoVirtual.P_SIT_RELAX,
            NaoVirtual.P_LYING_BELLY,
            NaoVirtual.P_LYING_BACK]

        for posture in posture_list:
            if posture.isPostureName(posture_name):
                self.setAngles(
                    posture.getPostureJointNames(),
                    posture.getPostureJointValues(),
                    percentage_speed)

                return True

        return False

    def subscribeCamera(self, camera_id, resolution=Camera.K_QVGA):
        """
        Subscribe to the camera holding the camera id. WARNING: at the moment,
        only one camera can be subscribed.

        Parameters:
            camera_id - The id of the camera to be subscribed
            resolution - CameraResolution object, the resolution of the camera
        """
        if camera_id == NaoVirtual.ID_CAMERA_TOP:
            self.camera_top.subscribe(resolution=resolution)

        elif camera_id == NaoVirtual.ID_CAMERA_BOTTOM:
            self.camera_bottom.subscribe(resolution=resolution)

    def unsubscribeCamera(self, camera_id):
        """
        Unsubscribe from a camera, the one holding the camera id.

        Parameters:
            camera_id - The id of the camera to be unsubscribed
        """
        if camera_id == NaoVirtual.ID_CAMERA_TOP:
            self.camera_top.unsubscribe()

        elif camera_id == NaoVirtual.ID_CAMERA_BOTTOM:
            self.camera_bottom.unsubscribe()

    def getCameraFrame(self):
        """
        Returns a camera frame. Be advised that the subscribeCamera method
        needs to be called beforehand.

        Returns:
            frame - The current camera frame as a formatted numpy array,
            directly exploitable from OpenCV
        """
        if self.camera_top.isActive():
            return self.camera_top.getFrame()
        elif self.camera_bottom.isActive():
            return self.camera_bottom.getFrame()

    def getCameraResolution(self):
        """
        Returns the resolution of the active camera. If no camera is active,
        returns None

        Returns:
            resolution - a CameraResolution object describing the resolution of
            the active camera
        """
        if self.camera_top.isActive():
            return self.camera_top.getResolution()
        elif self.camera_bottom.isActive():
            return self.camera_bottom.getResolution()

    def _mimicHand(
            self,
            hand,
            value,
            multiplier=0.999899,
            offset=0.0):
        """
        Used to propagate a joint value on the fingers attached to the hand.
        The formula used to mimic a joint is the following (with a multiplier
        of 0.999899 and an offset of 0.0):

        finger_value = (hand_value * multiplier) + offset

        Parameters:
            hand - String, RHand or LHand
            value - the joint value to be propagated

        Returns:
            finger_names - Names of the finger to be controlled
            finger_values - Values of the fingers to be controlled
        """
        finger_names = list()
        finger_values = list()

        for joint_name in self.joint_dict.keys():
            if (hand[0] + "Finger") in joint_name or\
               (hand[0] + "Thumb") in joint_name:
                finger_names.append(joint_name)
                finger_values.append((value * multiplier) + offset)

        return finger_names, finger_values
