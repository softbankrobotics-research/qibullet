#!/usr/bin/env python
# coding: utf-8

import os
import pybullet
from qibullet.laser import *
from qibullet.camera import *
from qibullet.base_controller import *
from qibullet.robot_posture import PepperPosture
from qibullet.robot_virtual import RobotVirtual


class PepperVirtual(RobotVirtual):
    """
    Class representing the virtual instance of the Pepper robot
    """
    ID_CAMERA_TOP = 0
    ID_CAMERA_BOTTOM = 1
    ID_CAMERA_DEPTH = 2
    FRAME_WORLD = 1
    FRAME_ROBOT = 2
    URDF_PATH = "robot_data/pepper_1.7/pepper.urdf"
    P_STAND = PepperPosture("Stand")
    P_STAND_INIT = PepperPosture("StandInit")
    P_STAND_ZERO = PepperPosture("StandZero")
    P_CROUCH = PepperPosture("Crouch")

    def __init__(self):
        """
        Constructor
        """
        RobotVirtual.__init__(self, PepperVirtual.URDF_PATH)
        self.motion_constraint = None
        # Default speed (in m/s) xy : 0.35, min : 0.1, max : 0.55
        self.linear_velocity = 0.35
        # Default speed (in rad/s) theta : 1.0, min : 0.3, max : 2.0
        self.angular_velocity = 1.0
        # Default acc (in m/s^2 xy : 0.3, min : 0.1, max : 0.55
        self.linear_acceleration = 0.3
        # Default acc (in rad/s^2 theta : 0.75, min : 0.1, max : 3.0
        self.angular_acceleration = 0.3

    def loadRobot(self, translation, quaternion, physicsClientId=0):
        """
        Overloads @loadRobot from the @RobotVirtual class, loads the robot into
        the simulated instance. This method also updates the max velocity of
        the robot's fingers, adds self collision filters to the model, adds a
        motion constraint to the model and defines the cameras of the model in
        the camera_dict.

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

        RobotVirtual.loadRobot(
            self,
            translation,
            quaternion,
            physicsClientId=physicsClientId)

        for base_link in ["Hip", "Pelvis"]:
            pybullet.setCollisionFilterPair(
                self.robot_model,
                self.robot_model,
                self.link_dict["torso"].getIndex(),
                self.link_dict[base_link].getIndex(),
                0,
                physicsClientId=self.physics_client)

        for shoulder_roll_link in ["RBicep", "LBicep"]:
            pybullet.setCollisionFilterPair(
                self.robot_model,
                self.robot_model,
                self.link_dict["torso"].getIndex(),
                self.link_dict[shoulder_roll_link].getIndex(),
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

        for joint_name in list(self.joint_dict):
            if 'RFinger' in joint_name or 'RThumb' in joint_name:
                self.joint_dict[joint_name].setMaxVelocity(
                    self.joint_dict["RHand"].getMaxVelocity())
            elif 'LFinger' in joint_name or 'LThumb' in joint_name:
                self.joint_dict[joint_name].setMaxVelocity(
                    self.joint_dict["LHand"].getMaxVelocity())
            elif "Wheel" in joint_name:
                self.joint_dict.pop(joint_name)

        camera_top = CameraRgb(
            self.robot_model,
            PepperVirtual.ID_CAMERA_TOP,
            self.link_dict["CameraTop_optical_frame"],
            hfov=56.3,
            vfov=43.7,
            physicsClientId=self.physics_client)

        camera_bottom = CameraRgb(
            self.robot_model,
            PepperVirtual.ID_CAMERA_BOTTOM,
            self.link_dict["CameraBottom_optical_frame"],
            hfov=56.3,
            vfov=43.7,
            physicsClientId=self.physics_client)

        camera_depth = CameraDepth(
            self.robot_model,
            PepperVirtual.ID_CAMERA_DEPTH,
            self.link_dict["CameraDepth_optical_frame"],
            hfov=58.0,
            vfov=45.0,
            physicsClientId=self.physics_client)

        self.camera_dict = {
            PepperVirtual.ID_CAMERA_TOP: camera_top,
            PepperVirtual.ID_CAMERA_BOTTOM: camera_bottom,
            PepperVirtual.ID_CAMERA_DEPTH: camera_depth}

        self.motion_constraint = pybullet.createConstraint(
            parentBodyUniqueId=self.robot_model,
            parentLinkIndex=-1,
            childBodyUniqueId=-1,
            childLinkIndex=-1,
            jointType=pybullet.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=[0, 0, 0],
            parentFrameOrientation=[0, 0, 0, 1],
            childFramePosition=[translation[0], translation[1], 0],
            childFrameOrientation=quaternion,
            physicsClientId=self.physics_client)

        self.laser_manager = Laser(
            self.robot_model,
            self.link_dict["Tibia"].getIndex(),
            physicsClientId=self.physics_client)

        self.base_controller = PepperBaseController(
            self.robot_model,
            [self.linear_velocity, self.angular_velocity],
            [self.linear_acceleration, self.angular_acceleration],
            self.motion_constraint,
            physicsClientId=self.physics_client)

        self.goToPosture("StandZero", 1.0)

    def moveTo(self, x, y, theta, frame=FRAME_ROBOT, speed=None, _async=False):
        """
        Move the robot in frame world or robot (FRAME_WORLD=1, FRAME_ROBOT=2).
        This method can be called synchonously or asynchronously. In the
        asynchronous mode, the function can be called when it's already
        launched, this will update the goal of the motion.

        Parameters:
            x - position of the goal on the x axis, in meters
            y - position of the goal on the y axis, in meters
            theta - orientation of the goal around the z axis, in radians
            frame - The frame in which the goal is expressed: FRAME_WORLD = 1,
            FRAME_ROBOT = 2
            speed - The desired linear velocity, in m/s
            _async - The method is launched in async mode if True, in synch
            mode if False (False by default)
        """
        if speed is not None:
            self.base_controller.setLinearVelocity(speed)

        self.base_controller.moveTo(x, y, theta, frame, _async=_async)

    def move(self, x, y, theta):
        """
        Apply a speed on the robot's base.

        Parameters:
            x - Speed on the x axis, in m/s
            y - Speed on the y axis, in m/s
            theta - Rotational speed around the z axis, in rad/s
        """
        self.base_controller.move(x, y, theta)

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
        available postures are PepperPosture objects.

        Parameters:
            posture_name - String containing the name of the posture. The
            posture name is not case-sensitive (The available postures
            are Stand or StandInit, StandZero and Crouch)
            percentage_speed - Percentage of the max speed to be used for the
            movement

        Returns:
            Boolean - True if the posture can be applied, False otherwise
        """
        posture_list = [
            PepperVirtual.P_STAND,
            PepperVirtual.P_STAND_INIT,
            PepperVirtual.P_STAND_ZERO,
            PepperVirtual.P_CROUCH]

        for posture in posture_list:
            if posture.isPostureName(posture_name):
                self.setAngles(
                    posture.getPostureJointNames(),
                    posture.getPostureJointValues(),
                    percentage_speed)

                return True

        return False

    def subscribeLaser(self):
        """
        Subscribe to the robot's lasers. Calling this method will launch the
        laser scan process: note that you need the laser scan to be enabled to
        successfully retrieve laser data
        """
        self.laser_manager.subscribe()

    def unsubscribeLaser(self):
        """
        Unsubscribe from the robot's lasers. Calling this method will stop the
        laser scan process
        """
        self.laser_manager.unsubscribe()

    def showLaser(self, display):
        """
        Display debug lines that simulate the laser
        """
        self.laser_manager.showLaser(display)

    def getFrontLaserValue(self):
        """
        Return a list of the front laser value (clockwise)
        """
        return self.laser_manager.getFrontLaserValue()

    def getRightLaserValue(self):
        """
        Return a list of the right laser value (clockwise)
        """
        return self.laser_manager.getRightLaserValue()

    def getLeftLaserValue(self):
        """
        Return a list of the left laser value (clockwise)
        """
        return self.laser_manager.getLeftLaserValue()

    def _mimicHand(self, hand, value, multiplier=0.872665, offset=0):
        """
        Used to propagate a joint value on the fingers attached to the hand.
        The formula used to mimic a joint is the following:

        finger_value = (hand_value * multiplier) + offset

        Parameters:
            hand - String, RHand or LHand
            value - The joint value to be propagated
            multiplier - The multiplier coefficient (0.872665 by default)
            offset - The offset coefficient (0.0 by default)

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
