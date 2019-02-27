#!/usr/bin/env python
# coding: utf-8

import time
import atexit
import weakref
import pybullet
import threading

from qibullet.tools import *


class BaseController(object):
    """
    Class controlling the robot base
    """
    _instances = set()

    def __init__(
                self,
                robot_model,
                speed, acc, physicsClientId=0):
        """
        Constructor

        Parameters:
            robot_model - the pybullet model of the robot.
            speed - list composed of velocity on xy and velocity on theta
            [vel_xy, vel_theta].
            acc - list composed of acceleration on xy and acceleration on theta
            [acc_xy, acc_theta].
            physicsClientId - The id of the simulated instance in which the
            robot will be controlled.
        """
        self.vel_xy, self.vel_theta = speed
        self.acc_xy, self.acc_theta = acc
        self.control_process = threading.Thread(target=None)
        self.physics_client = physicsClientId
        self.robot_model = robot_model
        self.frame = 2
        self.pose_init = {}
        self.pose_goal = {}
        self._instances.add(weakref.ref(self))
        self._controller_termination = False
        atexit.register(self._terminateController)

    @classmethod
    def _getInstances(cls):
        """
        INTERNAL CLASSMETHOD, get all of the BaseController (and daughters)
        instances
        """
        dead = set()

        for ref in cls._instances:
            obj = ref()

            if obj is not None:
                yield obj
            else:
                dead.add(ref)

        cls._instances -= dead

    def _setGoal(self, x, y, theta, frame):
        """
        INTERNAL METHOD, set the position of the goal on a specific frame.

        Parameters:
            x - float in meters.
            y - float in meters.
            theta - float in radians.
            frame - FRAME_WORLD = 1, FRAME_ROBOT = 2.
        """
        self.goal = [x, y, theta]
        self.frame = frame

    def _updateGoal(self):
        """
        INTERNAL METHOD, update the position of the goal.
        """
        # get actual position in frame world
        actual_pose, actual_orn = pybullet.getBasePositionAndOrientation(
            self.robot_model,
            physicsClientId=self.physics_client)
        x, y, theta = self.goal
        # pose x, y, z
        pose_requested = [x, y, 0]
        # orientation requested (quaternions)
        orn_requested = pybullet.getQuaternionFromEuler([0, 0, theta])
        # if we are in frame robot add position in the frame world
        if self.frame == 2:
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
        self.pose_goal["position"] = pose_requested
        self.pose_goal["orientation"] = orn_requested

    def setVelXY(self, vel_xy):
        """
        set the velocity on axis xy.

        Parameter:
            vel_xy : velocity on axis xy in m/s.
        """
        self.vel_xy = vel_xy

    def _terminateController(self):
        """
        INTERNAL METHOD, can be called to terminate an asynchronous controller
        """
        self._controller_termination = True

        if self.control_process.isAlive():
            self.control_process.join()


class PepperBaseController(BaseController):
    """
    Class Controlling the robot Pepper.
    """
    def __init__(
                self,
                robot_model,
                speed, acc, motion_constraint, physicsClientId=0):
        """
        Constructor

        Parameters:
            robot_model - the pybullet model of the robot.
            speed - list composed of velocity on xy and velocity on theta.
            acc - list composed of acceleration on xy and acceleration on
            theta.
            motion_constraint - the pybullet motion constraint applied on the
            robot.
            physicsClientId - The id of the simulated instance in which the
            Pepper will be controlled.
        """
        BaseController.__init__(
                        self,
                        robot_model,
                        speed,
                        acc,
                        physicsClientId=physicsClientId)
        # force applied in the movement
        self.force = 100
        # The robot will stop the movement with a precision
        # of 0.01 m and 0.02 rads
        self.threshold_xy = 0.01
        self.threshold_theta = 0.02
        self.motion_constraint = motion_constraint

    def moveTo(self, x, y, theta, frame, _async=False):
        """
        Move the robot in frame world or robot
        (FRAME_WORLD = 1, FRAME_ROBOT = 2). It can be launched synchonous or
        asynchronous. In the asynchronous mode, call the function when it's
        already launched will update the goal of the motion.

        Parameters:
            x - float in meters.
            y - float in meters.
            theta - float in radians.
            frame - FRAME_WORLD = 1, FRAME_ROBOT = 2.
            _async - boolean (initate at False by default)
        """
        self._setGoal(x, y, theta, frame)
        if self.control_process.isAlive():
            if _async is False:
                raise pybullet.error(
                        "Already a moveTo asynchronous."
                        " Can't launch moveTo synchronous")
            self._initProcess()
        elif _async:
            self.control_process = threading.Thread(target=self._moveToProcess)
            self.control_process.start()
        else:
            self._moveToProcess()

    def _updateConstraint(self):
        """
        INTERNAL METHOD, update the robot's constraint.
        """
        # change the constraint to the position and orientation requested
        pybullet.changeConstraint(
            self.motion_constraint,
            self.pose_goal["position"],
            jointChildFrameOrientation=self.pose_goal["orientation"],
            maxForce=self.force,
            physicsClientId=self.physics_client)

    def _initProcess(self):
        """
        INTERNAL METHOD, initialize the motion process and all variables
        needed.
        """
        # get actual position in frame world
        self.pose_init["position"], self.pose_init["orientation"] =\
            pybullet.getBasePositionAndOrientation(
                self.robot_model,
                physicsClientId=self.physics_client)
        self._updateGoal()
        self._updateConstraint()

        # Compute the ratio distance requested on distance total
        distance = getDistance(
                                self.pose_init["position"],
                                self.pose_goal["position"])
        self.p_x = 0
        self.p_y = 0
        self.p_theta = 0
        if distance:
            self.p_x =\
                (self.pose_goal["position"][0] -
                    self.pose_init["position"][0]) / distance
            self.p_y =\
                (self.pose_goal["position"][1] -
                    self.pose_init["position"][1]) / distance
        theta_to_do = getOrientation(
                                    self.pose_init["orientation"],
                                    self.pose_goal["orientation"])
        if abs(theta_to_do):
            self.p_theta = abs(theta_to_do) / theta_to_do

    def _endProcess(self):
        """
        INTERNAL METHOD, stop the robot movement.
        """
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
            maxForce=self.force * 10,
            physicsClientId=self.physics_client)
        pybullet.resetBaseVelocity(
            self.robot_model,
            [0, 0, 0],
            [0, 0, 0],
            physicsClientId=self.physics_client)

    def _moveToProcess(self):
        """
        INTERNAL METHOD, process to move the robot's base.
        """
        self._initProcess()
        actual_pose = self.pose_init["position"]
        actual_orn = self.pose_init["orientation"]

        while not self._controller_termination:
            translation_distance = getDistance(
                actual_pose, self.pose_goal["position"])
            rotation_distance = abs(getOrientation(
                actual_orn,
                self.pose_goal["orientation"]))

            if translation_distance > self.threshold_xy or\
                    rotation_distance > self.threshold_theta:
                break

            actual_pose, actual_orn = pybullet.getBasePositionAndOrientation(
                self.robot_model,
                physicsClientId=self.physics_client)
            vel_x = computeVelocity(
                        self.acc_xy,
                        0.05,
                        self.vel_xy,
                        getDistance(
                            self.pose_init["position"], actual_pose),
                        getDistance(
                            actual_pose, self.pose_goal["position"])
                        )
            vel_y = vel_x
            vel_theta = computeVelocity(
                        self.acc_theta,
                        0.05,
                        self.vel_theta,
                        abs(getOrientation(
                            self.pose_init["orientation"],
                            self.pose_goal["orientation"])),
                        abs(getOrientation(
                            actual_orn, self.pose_goal["orientation"]))
                        )
            # if the robot is on the position requested, we set the
            # velocity to 0.
            if abs(actual_pose[0] - self.pose_goal["position"][0]) <=\
                    self.threshold_xy / 2:
                vel_x = 0
            if abs(actual_pose[1] - self.pose_goal["position"][1]) <=\
                    self.threshold_xy / 2:
                vel_y = 0
            if abs(getOrientation(
                    actual_orn, self.pose_goal["orientation"])) <=\
                    self.threshold_theta:
                vel_theta = 0
            # reset velocity of the robot
            time.sleep(0.02)
            pybullet.resetBaseVelocity(
                self.robot_model,
                [vel_x * self.p_x, vel_y * self.p_y, 0],
                [0, 0, vel_theta * self.p_theta],
                physicsClientId=self.physics_client)

        self._endProcess()
