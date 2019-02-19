#!/usr/bin/env python
# coding: utf-8

import os
import time
import pybullet
from qibullet.tools import *
from qibullet.camera import *
from qibullet.robot_posture import PepperPosture
from qibullet.robot_virtual import RobotVirtual

MAX_VEL_XY = 0.55
MIN_VEL_XY = 0.1
MAX_ACC_XY = 0.55
MIN_ACC_XY = 0.3
MAX_VEL_THETA = 2.0
MIN_VEL_THETA = 0.3
MAX_ACC_THETA = 3.0
MIN_ACC_THETA = 0.75


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
        self.camera_top = None
        self.camera_bottom = None
        self.camera_depth = None
        self.motion_constraint = None
        # Default speed (in m/s) xy : 0.35, min : 0.1, max : 0.55
        self.vel_xy = 0.35
        # Default acc (in m/s^2 xy : 0.3, min : 0.1, max : 0.55
        self.acc_xy = 0.3
        # Default speed (in rad/s) theta : 1.0, min : 0.2, max : 2.0
        self.vel_theta = 1.0
        # Default acc (in rad/s^2 theta : 0.75, min : 0.1, max : 3.0
        self.acc_theta = 0.3

    def loadRobot(self, translation, quaternion, physicsClientId=0):
        """
        Overload @loadRobot from the @RobotVirtual class. Update max velocity
        for the fingers and thumbs, based on the hand joints. Add self
        collision exceptions (The biceps won't autocollide with the torso, the
        fingers and thumbs of a hand won't autocollide with the corresponding
        wrist). Add the cameras. Add motion constraint.
        """
        pybullet.setAdditionalSearchPath(
            os.path.dirname(os.path.realpath(__file__)),
            physicsClientId=physicsClientId)

        RobotVirtual.loadRobot(
            self,
            translation,
            quaternion,
            physicsClientId=physicsClientId)

        for joint_name in list(self.joint_dict):
            if 'RFinger' in joint_name or 'RThumb' in joint_name:
                self.joint_dict[joint_name].setMaxVelocity(
                    self.joint_dict["RHand"].getMaxVelocity())
            elif 'LFinger' in joint_name or 'LThumb' in joint_name:
                self.joint_dict[joint_name].setMaxVelocity(
                    self.joint_dict["LHand"].getMaxVelocity())
            elif "Wheel" in joint_name:
                self.joint_dict.pop(joint_name)

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

        self.camera_top = CameraRgb(
            self.robot_model,
            self.link_dict["CameraTop_optical_frame"],
            physicsClientId=self.physics_client)

        self.camera_bottom = CameraRgb(
            self.robot_model,
            self.link_dict["CameraBottom_optical_frame"],
            physicsClientId=self.physics_client)

        self.camera_depth = CameraDepth(
            self.robot_model,
            self.link_dict["CameraDepth_optical_frame"],
            physicsClientId=self.physics_client)

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

    def moveTo(self, x, y, theta, frame=FRAME_ROBOT, speed=None):
        """
        Move the robot in frame world or robot.
        (FRAME_WORLD = 1, FRAME_ROBOT = 2)

        Parameters:
            x - float in meters
            y - float in meters
            theta - float in radians
        """

        # force applied in the movement
        force = 100
        # The robot will stop the movement with a precision
        # of 0.01 m and 0.02 rads
        threshold_xy = 0.01
        threshold_theta = 0.02

        # get actual position in frame world
        actual_pose, actual_orn = pybullet.getBasePositionAndOrientation(
            self.robot_model,
            physicsClientId=self.physics_client)
        # pose x, y, z
        pose_requested = [x, y, 0]
        # orientation requested (quaternions)
        orn_requested = pybullet.getQuaternionFromEuler([0, 0, theta])
        # if we are in frame robot add position in the frame world
        if frame == 2:
            orn_euler = pybullet.getEulerFromQuaternion(actual_orn)
            pose_requested = [
                pose_requested[0] * math.cos(orn_euler[2])
                - pose_requested[1] * math.sin(orn_euler[2])
                + actual_pose[0],
                pose_requested[0] * math.sin(orn_euler[2])
                + pose_requested[1] * math.cos(orn_euler[2])
                + actual_pose[1],
                0]
            orn_requested = pybullet.getQuaternionFromEuler(
                [orn_euler[0],
                 orn_euler[1],
                 orn_euler[2] + theta])
        # change the constraint to the position and orientation requested
        pybullet.changeConstraint(
            self.motion_constraint,
            pose_requested,
            jointChildFrameOrientation=orn_requested,
            maxForce=force,
            physicsClientId=self.physics_client)
        # init robot speed
        speed_xy = self.vel_xy
        if speed is not None:
            speed_xy = speed
        vel_x, vel_y, vel_theta = [speed_xy, speed_xy, self.vel_theta]
        # Compute the ratio distance requested on distance total
        distance = getDistance(actual_pose, pose_requested)
        p_x = 0
        p_y = 0
        p_theta = 0
        if distance:
            p_x = (pose_requested[0] - actual_pose[0]) / distance
            p_y = (pose_requested[1] - actual_pose[1]) / distance
        theta_to_do = getOrientation(actual_orn, orn_requested)
        if abs(theta_to_do):
            p_theta = abs(theta_to_do) / theta_to_do

        pose_init = actual_pose
        orn_init = actual_orn
        while getDistance(actual_pose, pose_requested) > threshold_xy\
                or abs(getOrientation(actual_orn, orn_requested)) >\
                threshold_theta:

            actual_pose, actual_orn = pybullet.getBasePositionAndOrientation(
                self.robot_model,
                physicsClientId=self.physics_client)
            vel_x = computeVelocity(
                        self.acc_xy,
                        0.05,
                        speed_xy,
                        getDistance(pose_init, actual_pose),
                        getDistance(actual_pose, pose_requested)
                        )
            vel_y = vel_x
            vel_theta = computeVelocity(
                        self.acc_theta,
                        0.05,
                        self.vel_theta,
                        abs(getOrientation(orn_init, orn_requested)),
                        abs(getOrientation(actual_orn, orn_requested))
                        )
            # if the robot is on the position requested, we set the
            # velocity to 0.
            if abs(actual_pose[0] - pose_requested[0]) <= threshold_xy / 2:
                vel_x = 0
            if abs(actual_pose[1] - pose_requested[1]) <= threshold_xy / 2:
                vel_y = 0
            if abs(getOrientation(actual_orn, orn_requested)) <=\
                    threshold_theta:
                vel_theta = 0
            # reset velocity of the robot
            time.sleep(0.02)
            pybullet.resetBaseVelocity(
                self.robot_model,
                [vel_x * p_x, vel_y * p_y, 0],
                [0, 0, vel_theta * p_theta],
                physicsClientId=self.physics_client)
        # Change the constraint to the actual position and orientation in
        # order to stop the robot's motion. The force applied is huge
        # to avoid oscillation.
        actual_pose, actual_orn = pybullet.getBasePositionAndOrientation(
            self.robot_model,
            physicsClientId=self.physics_client)
        pybullet.changeConstraint(
            self.motion_constraint,
            actual_pose,
            jointChildFrameOrientation=actual_orn,
            maxForce=force * 10,
            physicsClientId=self.physics_client)
        pybullet.resetBaseVelocity(
            self.robot_model,
            [0, 0, 0],
            [0, 0, 0],
            physicsClientId=self.physics_client)

    def setAngles(self, joint_names, joint_values, percentage_speed):
        """
        Overload @setAngles from the @RobotVirtual class. Handles the finger
        mimic behaviour.

        Parameters:
            joint_names - List of string (or string if only one joint)
            containing the name of the joints to be controlled
            joint_values - List of values (or value if only one joint)
            corresponding to the angles in radians to be applied
            percentage_speed - Percentage of the max speed to be used for the
            movement
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

        except AssertionError:
            print("Error in the parameters given to the setAngles method")
            return

        for hand in ["RHand", "LHand"]:
            for i in range(names.count(hand)):
                index = names.index(hand)
                value = values[index]
                names.pop(index)
                values.pop(index)
                finger_names, finger_values = self._mimicHand(hand, value)
                names.extend(finger_names)
                values.extend(finger_values)

        RobotVirtual.setAngles(
            self,
            names,
            values,
            percentage_speed)

    def getAnglesPosition(self, joint_names):
        """
        Overload @getAnglesPosition from the @RobotVirtual class. Handles the
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

    def setVelXY(self, value):
        """
        Set xy velocity of the robot

        Parameter:
            value of the velocity in m/s
        """
        if value >= MAX_VEL_XY:
            self.vel_xy = MAX_VEL_XY
        elif value <= MIN_VEL_XY:
            self.vel_xy = MIN_VEL_XY
        else:
            self.vel_xy = value

    def setVelTheta(self, value):
        """
        Set theta velocity of the robot

        Parameter:
            value of the velocity in m/s
        """
        if value >= MAX_VEL_THETA:
            self.vel_theta = MAX_VEL_THETA
        elif value <= MIN_VEL_THETA:
            self.vel_theta = MIN_VEL_THETA
        else:
            self.vel_theta = value

    def setAccXY(self, value):
        """
        Set xy acceleration of the robot

        Parameter:
            value of the velocity in m/s^2
        """
        if value >= MAX_ACC_XY:
            self.acc_xy = MAX_ACC_XY
        elif value <= MIN_ACC_XY:
            self.acc_xy = MIN_ACC_XY
        else:
            self.acc_xy = value

    def setAccTheta(self, value):
        """
        Set theta acceleration of the robot

        Parameter:
            value of the velocity in rad/s^2
        """
        if value >= MAX_ACC_THETA:
            self.acc_theta = MAX_ACC_THETA
        elif value <= MIN_ACC_THETA:
            self.acc_theta = MIN_ACC_THETA
        else:
            self.acc_theta = value

    def getMaxVelXY(self):
        return MAX_VEL_XY

    def getMinVelXY(self):
        return MIN_VEL_XY

    def getMaxVelTheta(self):
        return MAX_VEL_THETA

    def getMinVelTheta(self):
        return MIN_VEL_THETA

    def subscribeCamera(self, camera_id, resolution=Camera.K_QVGA):
        """
        Subscribe to the camera holding the camera id. WARNING: at the moment,
        only one camera can be subscribed.

        Parameters:
            camera_id - The id of the camera to be subscribed
            resolution - CameraResolution object, the resolution of the camera
        """
        if camera_id == PepperVirtual.ID_CAMERA_TOP:
            self.camera_top.subscribe(resolution=resolution)

        elif camera_id == PepperVirtual.ID_CAMERA_BOTTOM:
            self.camera_bottom.subscribe(resolution=resolution)

        elif camera_id == PepperVirtual.ID_CAMERA_DEPTH:
            self.camera_depth.subscribe(resolution=resolution)

    def unsubscribeCamera(self, camera_id):
        """
        Unsubscribe from a camera, the one holding the camera id.

        Parameters:
            camera_id - The id of the camera to be unsubscribed
        """
        if camera_id == PepperVirtual.ID_CAMERA_TOP:
            self.camera_top.unsubscribe()

        elif camera_id == PepperVirtual.ID_CAMERA_BOTTOM:
            self.camera_bottom.unsubscribe()

        elif camera_id == PepperVirtual.ID_CAMERA_DEPTH:
            self.camera_depth.unsubscribe()

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
        elif self.camera_depth.isActive():
            return self.camera_depth.getFrame()

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
        elif self.camera_depth.isActive():
            return self.camera_depth.getResolution()

    def isSelfColliding(self, link_names):
        """
        Specifies if a link is colliding with the rest of the virtual Pepper
        robot.

        Parameters:
            link_names - String or list of string containing the names of the
            links to be checked for self collision. WARNING: only the links
            with corresponding meshes should be used, otherwise the link cannot
            self collide

        Returns:
            self_colliding - Boolean, if True at least one of the links is self
            colliding
        """
        try:
            if type(link_names) is str:
                assert link_names in self.link_dict.keys()
                names = [link_names]
            else:
                assert set(link_names).issubset(self.link_dict.keys())
                names = list(link_names)

            for name in names:
                contact_tuple = pybullet.getContactPoints(
                    bodyA=self.robot_model,
                    bodyB=self.robot_model,
                    linkIndexA=self.link_dict[name].getIndex(),
                    physicsClientId=self.physics_client)
                contact_tuple += pybullet.getContactPoints(
                    bodyA=self.robot_model,
                    bodyB=self.robot_model,
                    linkIndexB=self.link_dict[name].getIndex(),
                    physicsClientId=self.physics_client)

                if len(contact_tuple) != 0:
                    return True

            return False

        except AssertionError:
            "Unauthorized link checking for self collisions"
            return False

    def _mimicHand(self, hand, value, multiplier=0.872665, offset=0):
        """
        Used to propagate a joint value on the fingers attached to the hand.
        The formula used to mimic a joint is the following (with a multiplier
        of 0.872665 and an offset of 0):

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
