#!/usr/bin/env python
# coding: utf-8

import time
import atexit
import weakref
import pybullet
import threading

from qibullet.tools import *
from qibullet.controller import Controller


class BaseController(Controller):
    """
    Class describing a robot base controller
    """
    # _instances = set()
    FRAME_WORLD = 1
    FRAME_ROBOT = 2

    def __init__(self, robot_model, physicsClientId=0):
        """
        Constructor

        Parameters:
            robot_model - the pybullet model of the robot
            physicsClientId - The id of the simulated instance in which the
            robot will be controlled
        """
        Controller.__init__(self, robot_model, physicsClientId)
        self.linear_velocity = 0
        self.angular_velocity = 0
        self.linear_acceleration = 0
        self.angular_acceleration = 0
        self.frame = BaseController.FRAME_ROBOT
        self.pose_init = {}
        self.pose_goal = {}

    def _setGoal(self, x, y, theta, frame):
        """
        INTERNAL METHOD, set the position of the goal to a specific frame.

        Parameters:
            x - position of the goal on the x axis, in meters
            y - position of the goal on the y axis, in meters
            theta - orientation of the goal around the z axis, in radians
            frame - The frame in which the goal is expressed: FRAME_WORLD = 1,
            FRAME_ROBOT = 2
        """
        self.goal = [x, y, theta]
        self.frame = frame

    def _updateGoal(self):
        """
        INTERNAL METHOD, update the position of the goal.
        """
        # get actual position in frame world
        actual_pos, actual_orn = pybullet.getBasePositionAndOrientation(
            self.robot_model,
            physicsClientId=self.physics_client)

        x, y, theta = self.goal
        # pose x, y, z
        pose_requested = [x, y, 0]

        # orientation requested (euler)
        orn_requested = [0, 0, theta]
        # if we are in frame robot express the position in the frame world
        if self.frame == BaseController.FRAME_ROBOT:
            orn_euler = pybullet.getEulerFromQuaternion(actual_orn)
            pose_requested = [
                pose_requested[0] * math.cos(orn_euler[2])
                - pose_requested[1] * math.sin(orn_euler[2])
                + actual_pos[0],
                pose_requested[0] * math.sin(orn_euler[2])
                + pose_requested[1] * math.cos(orn_euler[2])
                + actual_pos[1],
                0]
            orn_requested = [
                orn_euler[0],
                orn_euler[1],
                orn_euler[2] + theta]
        self.pose_goal["position"] = pose_requested
        self.pose_goal["orientation"] = orn_requested

    def setLinearVelocity(self, linear_velocity):
        """
        Set the linear velocity.

        Parameter:
            linear_velocity : The linear velocity value in m/s
        """
        self.linear_velocity = linear_velocity

    def _setAngularVelocity(self, angular_velocity):
        """
        INTERNAL METHOD, set the angular velocity.

        Parameter:
            angular_velocity : The angular velocity value in rad/s
        """
        self.angular_velocity = angular_velocity

    def _setLinearAcceleration(self, linear_acceleration):
        """
        INTERNAL METHOD, set the linear acceleration.

        Parameter:
            linear_acceleration : The linear acceleration value in m/s^2
        """
        self.linear_acceleration = linear_acceleration

    def _setAngularAcceleration(self, angular_acceleration):
        """
        INTERNAL METHOD, set the angular acceleration.

        Parameter:
            angular_acceleration : The angular acceleration value in rad/s^2
        """
        self.angular_acceleration = angular_acceleration


class PepperBaseController(BaseController):
    """
    Class describing a Pepper base controller
    """
    MAX_LINEAR_VELOCITY = 0.55
    MIN_LINEAR_VELOCITY = 0.1
    MAX_ANGULAR_VELOCITY = 2.0
    MIN_ANGULAR_VELOCITY = 0.3
    MAX_LINEAR_ACCELERATION = 0.55
    MIN_LINEAR_ACCELERATION = 0.1
    MAX_ANGULAR_ACCELERATION = 3.0
    MIN_ANGULAR_ACCELERATION = 0.1

    def __init__(
            self,
            robot_model,
            speed,
            acceleration,
            motion_constraint,
            physicsClientId=0):
        """
        Constructor

        Parameters:
            robot_model - the pybullet model of the robot
            speed - list containing the linear velocity and the angular
            velocity values, in m/s
            acceleration - list containing the linear acceleration and angular
            acceleration values, in m/s^2
            motion_constraint - the pybullet motion constraint applied on the
            robot
            physicsClientId - The id of the simulated instance in which Pepper
            will be controlled
        """
        BaseController.__init__(
            self,
            robot_model,
            physicsClientId=physicsClientId)

        # Set the different speeds and accelerations
        self.setLinearVelocity(speed[0])
        self._setAngularVelocity(speed[1])
        self._setLinearAcceleration(acceleration[0])
        self._setAngularAcceleration(acceleration[1])

        # force applied in the movement
        self.force = 100

        # The robot will stop the movement with a precisio of 0.01 m and 0.02
        # rads
        self.linear_threshold = 0.01
        self.angular_threshold = 0.02
        self.motion_constraint = motion_constraint

    def setLinearVelocity(self, linear_velocity):
        """
        Set the linear velocity.

        Parameter:
            linear_velocity : The linear velocity value in m/s
        """
        if linear_velocity > PepperBaseController.MAX_LINEAR_VELOCITY:
            linear_velocity = PepperBaseController.MAX_LINEAR_VELOCITY

        elif linear_velocity < PepperBaseController.MIN_LINEAR_VELOCITY:
            linear_velocity = PepperBaseController.MIN_LINEAR_VELOCITY

        BaseController.setLinearVelocity(self, linear_velocity)

    def _setAngularVelocity(self, angular_velocity):
        """
        INTERNAL METHOD, set the angular velocity.

        Parameter:
            angular_velocity : The angular velocity value in rad/s
        """
        if angular_velocity > PepperBaseController.MAX_ANGULAR_VELOCITY:
            angular_velocity = PepperBaseController.MAX_ANGULAR_VELOCITY

        elif angular_velocity < PepperBaseController.MIN_ANGULAR_VELOCITY:
            angular_velocity = PepperBaseController.MIN_ANGULAR_VELOCITY

        BaseController._setAngularVelocity(self, angular_velocity)

    def _setLinearAcceleration(self, linear_acceleration):
        """
        INTERNAL METHOD, set the linear acceleration.

        Parameter:
            linear_acceleration : The linear acceleration value in m/s^2
        """
        if linear_acceleration > PepperBaseController.MAX_LINEAR_ACCELERATION:
            linear_acceleration = PepperBaseController.MAX_LINEAR_ACCELERATION

        elif linear_acceleration <\
                PepperBaseController.MIN_LINEAR_ACCELERATION:
            linear_acceleration = PepperBaseController.MIN_LINEAR_ACCELERATION

        BaseController._setLinearAcceleration(self, linear_acceleration)

    def _setAngularAcceleration(self, angular_acceleration):
        """
        INTERNAL METHOD, set the angular acceleration.

        Parameter:
            angular_acceleration : The angular acceleration value in rad/s^2
        """
        if angular_acceleration >\
                PepperBaseController.MAX_ANGULAR_ACCELERATION:
            angular_acceleration =\
                PepperBaseController.MAX_ANGULAR_ACCELERATION

        elif angular_acceleration <\
                PepperBaseController.MIN_ANGULAR_ACCELERATION:
            angular_acceleration =\
                PepperBaseController.MIN_ANGULAR_ACCELERATION

        BaseController._setAngularAcceleration(self, angular_acceleration)

    def moveTo(self, x, y, theta, frame, _async=False):
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
            _async - The method is launched in async mode if True, in synch
            mode if False (False by default)
        """
        self._setGoal(x, y, theta, frame)

        if self.module_process.isAlive():
            if _async is False:
                raise pybullet.error(
                    "Already a moveTo asynchronous. Can't "
                    "launch moveTo synchronous")
            self._initProcess()
        elif _async:
            self.module_process = threading.Thread(target=self._moveToProcess)
            self.module_process.start()
        else:
            self._moveToProcess()

    def move(self, x, y, theta):
        """
        Apply a speed on the robot's base.

        Parameters:
            x - Speed on the x axis, in m/s
            y - Speed on the y axis, in m/s
            theta - Rotational speed around the z axis, in rad/s
        """
        # Kill any previous moveTo process running
        self.moveTo(0, 0, 0, frame=BaseController.FRAME_ROBOT, _async=True)

        # Bound the velocity. The max acceleration is not taken into account
        # here, this is a potential improvment
        if abs(x) > PepperBaseController.MAX_LINEAR_VELOCITY:
            x = PepperBaseController.MAX_LINEAR_VELOCITY * (x/abs(x))
        if abs(y) > PepperBaseController.MAX_LINEAR_VELOCITY:
            y = PepperBaseController.MAX_LINEAR_VELOCITY * (y/abs(y))
        if abs(theta) > PepperBaseController.MAX_ANGULAR_VELOCITY:
            theta = PepperBaseController.MAX_ANGULAR_VELOCITY *\
                (theta/abs(theta))

        actual_pos, actual_orn = pybullet.getBasePositionAndOrientation(
            self.robot_model,
            physicsClientId=self.physics_client)

        # convert actual_orn into euler
        actual_orn = pybullet.getEulerFromQuaternion(actual_orn)

        linear_world_velocity = [
            x * math.cos(actual_orn[2]) - y * math.sin(actual_orn[2]),
            x * math.sin(actual_orn[2]) + y * math.cos(actual_orn[2]),
            0]

        time.sleep(0.02)
        pybullet.resetBaseVelocity(
            self.robot_model,
            linear_world_velocity,
            [0, 0, theta],
            physicsClientId=self.physics_client)

    def _updateConstraint(self):
        """
        INTERNAL METHOD, update the robot's constraint.
        """
        # Change the constraint to the requested position and orientation
        pybullet.changeConstraint(
            self.motion_constraint,
            self.pose_goal["position"],
            jointChildFrameOrientation=pybullet.getQuaternionFromEuler(
                self.pose_goal["orientation"]),
            maxForce=self.force,
            physicsClientId=self.physics_client)

    def _initProcess(self):
        """
        INTERNAL METHOD, initialize the motion process and all variables
        needed.
        """
        # Get actual position in frame world
        self.pose_init["position"], self.pose_init["orientation"] =\
            pybullet.getBasePositionAndOrientation(
                self.robot_model,
                physicsClientId=self.physics_client)

        # convert pose_init orientation in orn_euler
        self.pose_init["orientation"] = pybullet.getEulerFromQuaternion(
            self.pose_init["orientation"]
        )
        self._updateGoal()
        self._updateConstraint()

        # Compute the ratio distance requested on the total distance
        distance = getDistance(
            self.pose_init["position"],
            self.pose_goal["position"])

        self.p_x = 0
        self.p_y = 0
        self.p_theta = 0

        if distance:
            self.p_x = (
                self.pose_goal["position"][0] -
                self.pose_init["position"][0]) / distance
            self.p_y = (
                self.pose_goal["position"][1] -
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
        # order to stop the robot's motion. The force applied is purposely huge
        # to avoid oscillations.
        actual_pos, actual_orn = pybullet.getBasePositionAndOrientation(
            self.robot_model,
            physicsClientId=self.physics_client)
        pybullet.changeConstraint(
            self.motion_constraint,
            actual_pos,
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
        INTERNAL METHOD, process allowing to move the robot's base.
        """
        self._initProcess()
        # actual_pos = self.pose_init["position"]
        # actual_orn = self.pose_init["orientation"]
        init_pos = self.pose_init["position"]
        init_orn = self.pose_init["orientation"]
        actual_pos = init_pos
        actual_orn = init_orn

        while not self._module_termination:
            translation_distance = getDistance(
                actual_pos,
                self.pose_goal["position"])

            # Modulo the orientation pose goal with conversion in quaternion
            modulo_quater_pose_goal = pybullet.getQuaternionFromEuler(
                self.pose_goal["orientation"])
            # Conversion into euler
            modulo_euler_pose_goal = pybullet.getEulerFromQuaternion(
                modulo_quater_pose_goal)
            rotation_distance = abs(getOrientation(
                actual_orn,
                modulo_euler_pose_goal))

            if translation_distance < self.linear_threshold and\
                    rotation_distance < self.angular_threshold:
                break

            actual_pos, actual_orn = pybullet.getBasePositionAndOrientation(
                self.robot_model,
                physicsClientId=self.physics_client)
            # convert actual_orn into euler
            actual_orn = pybullet.getEulerFromQuaternion(actual_orn)

            linear_vel_x = computeVelocity(
                self.linear_acceleration,
                0.05,
                self.linear_velocity,
                getDistance(actual_pos, init_pos),
                getDistance(actual_pos, self.pose_goal["position"]))

            linear_vel_y = linear_vel_x

            angular_vel = computeVelocity(
                self.angular_acceleration,
                0.05,
                self.angular_velocity,
                abs(getOrientation(
                    init_orn,
                    actual_orn)),
                abs(getOrientation(
                    actual_orn,
                    self.pose_goal["orientation"])))

            # If the robot is on the requested position, we set the velocity to
            # 0.
            if abs(actual_pos[0] - self.pose_goal["position"][0]) <=\
                    self.linear_threshold / 2:
                linear_vel_x = 0

            if abs(actual_pos[1] - self.pose_goal["position"][1]) <=\
                    self.linear_threshold / 2:
                linear_vel_y = 0

            if abs(getOrientation(
                    actual_orn, self.pose_goal["orientation"])) <=\
                    self.angular_threshold:
                angular_vel = 0

            # Reset the velocity of the robot
            time.sleep(0.02)
            pybullet.resetBaseVelocity(
                self.robot_model,
                [linear_vel_x * self.p_x, linear_vel_y * self.p_y, 0],
                [0, 0, angular_vel * self.p_theta],
                physicsClientId=self.physics_client)

        self._endProcess()
