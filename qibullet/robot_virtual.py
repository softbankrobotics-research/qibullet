#!/usr/bin/env python
# coding: utf-8

import sys
import pybullet
from qibullet.camera import *
from qibullet.link import Link
from qibullet.joint import Joint
from qibullet.tools import isNan

IS_VERSION_PYTHON_3 = sys.version_info[0] >= 3


class RobotVirtual:
    """
    Mother class representing a virtual robot
    """

    def __init__(self, description_file):
        """
        Constructor

        Parameters:
            description_file - The file giving the description of the virtual
            robot. For now, only URDF is handled
        """
        self.description_file = description_file
        self.physics_client = 0
        self.camera_dict = dict()
        self.joint_dict = dict()
        self.link_dict = dict()
        self.fsr_handler = None
        self.imu = None

    def loadRobot(self, translation, quaternion, physicsClientId=0):
        """
        Loads the robot into a simulation, loads the joints and the links
        descriptions. The joints are set to 0 rad.

        Parameters:
            translation - List containing 3 elements, the translation [x, y, z]
            of the robot in the WORLD frame
            quaternion - List containing 4 elements, the quaternion
            [x, y, z, q] of the robot in the WORLD frame
            physicsClientId - The id of the simulated instance in which the
            robot is supposed to be loaded
        """
        try:
            self.physics_client = physicsClientId
            self.robot_model = pybullet.loadURDF(
                self.description_file,
                translation,
                quaternion,
                useFixedBase=False,
                globalScaling=1.0,
                physicsClientId=self.physics_client,
                flags=pybullet.URDF_USE_SELF_COLLISION |
                pybullet.URDF_USE_MATERIAL_COLORS_FROM_MTL)

        except pybullet.error as e:
            raise pybullet.error("Cannot load robot model: " + str(e))

        for i in range(pybullet.getNumJoints(
                self.robot_model,
                physicsClientId=self.physics_client)):

            joint_info = pybullet.getJointInfo(
                self.robot_model,
                i,
                physicsClientId=self.physics_client)

            # PYTHON 3 version needs a conversion bytes to str
            if IS_VERSION_PYTHON_3:
                link_name = joint_info[12].decode('utf-8')
                joint_name = joint_info[1].decode('utf-8')
            else:
                link_name = joint_info[12]
                joint_name = joint_info[1]

            self.link_dict[link_name] = Link(joint_info)

            if joint_info[2] == pybullet.JOINT_PRISMATIC or\
                    joint_info[2] == pybullet.JOINT_REVOLUTE:
                self.joint_dict[joint_name] = Joint(joint_info)

    def getRobotModel(self):
        """
        Returns the pybullet model to which the module is associated.

        Returns:
            robot_model - The pybullet model of the robot
        """
        return self.robot_model

    def getPhysicsClientId(self):
        """
        Returns the id of the simulated instance in which the module is loaded.

        Returns:
            physics_client - The id of the simulation in which the robot
            (possessing the module) is spawned
        """
        return self.physics_client

    def getJoint(self, joint_name):
        """
        Returns the Joint object named "joint_name". If the required joint does
        not exist, the method will raise a KeyError.

        Parameters:
            joint_name - The name of the desired joint, eg "RShoulderPitch"
        """
        return self.joint_dict[joint_name]

    def getLink(self, link_name):
        """
        Returns the Link object named "link_name". If the required link does
        not exist, the method will raise a KeyError.

        Parameters:
            link_name - The name of the desired link, eg "Tibia"
        """
        return self.link_dict[link_name]

    def setAngles(self, joint_names, joint_values, percentage_speeds):
        """
        Set angles on the robot's joints. Tests have to be performed by the
        child class to guarantee the validity of the input parameters. If one
        of the specified joint values or percentage speed is a NaN, a pybullet
        error will be raised. Aditionally, if the joint_names, joint_values or
        percentage_speeds don't have the same size, a pybullet error will be
        raised

        Parameters:
            joint_names - List of string containing the name of the joints
            to be controlled
            joint_values - List of values corresponding to the angles in
            radians to be applied
            percentage_speeds - Percentages of the max speed to be used for
            each joint, has to be strictly superior to 0 and inferior or equal
            to 1
        """
        try:
            assert len(joint_names) ==\
                len(joint_values) ==\
                len(percentage_speeds)

            assert all(
                speed >= 0.0 and speed <= 1.0 for speed in percentage_speeds)

        except AssertionError:
            raise pybullet.error("Error in the setAngles parameters")

        for joint_name, joint_value, percentage_speed in zip(
                joint_names,
                joint_values,
                percentage_speeds):
             
            if isNan(joint_value):
                raise pybullet.error(
                    "Joint value specified for " + joint_name + " is a NaN")

            joint_speed =\
                self.joint_dict[joint_name].getMaxVelocity() *\
                percentage_speed

            pybullet.setJointMotorControl2(
                self.robot_model,
                self.joint_dict[joint_name].getIndex(),
                pybullet.POSITION_CONTROL,
                targetPosition=joint_value,
                maxVelocity=joint_speed,
                force=self.joint_dict[joint_name].getMaxEffort(),
                physicsClientId=self.physics_client)

    def getAnglesPosition(self, joint_names):
        """
        Gets the position of the robot's joints in radians. If one of the joint
        doesn't exist, the method will raise a KeyError.

        Parameters:
            joint_names - List of string containing the names of the joints

        Returns:
            joint_positions - List of floats containing the joint's positions
        """
        indexes = [self.joint_dict[name].getIndex() for name in joint_names]

        return [state[0] for state in pybullet.getJointStates(
            self.robot_model,
            indexes,
            physicsClientId=self.physics_client)]

    def getAnglesVelocity(self, joint_names):
        """
        Gets the velocity of the robot's joints in rad/s. If one of the joint
        doesn't exist, the method will raise a KeyError.

        Parameters:
            joint_names - List of string containing the names of the joints

        Returns:
            joint_velocities - List of floats containing the joint's velocities
        """
        indexes = [self.joint_dict[name].getIndex() for name in joint_names]

        return [state[1] for state in pybullet.getJointStates(
            self.robot_model,
            indexes,
            physicsClientId=self.physics_client)]

    def getLinkPosition(self, link_name):
        """
        Returns the position of a robot link in the WORLD frame. The position
        is expressed as a 3 dimensional translation [x, y, z] and a 4
        dimensional quaternion [x, y, z, w]. If the required link does not
        exist, the method will raise a KeyError.

        Parameters:
            link_name - The name of the desired link, eg "Tibia"

        Returns:
            translation - 3 dimensional list containing the translation
            quaternion - 4 dimensional list containing the rotation, as a
            quaternion
        """
        link_state = pybullet.getLinkState(
            self.robot_model,
            self.link_dict[link_name].getIndex(),
            physicsClientId=self.physics_client)

        return link_state[4], link_state[5]

    def subscribeCamera(self, camera_id, resolution=Camera.K_QVGA, fps=30.0):
        """
        Subscribe to the camera holding the camera id (for instance,
        PepperVirtual.ID_CAMERA_TOP). This method returns a handle for the
        camera, needed to get frames, the resolution / link of the camera and
        the camera object. Bear in mind that when subscribing, a background
        process is started continuously retrieving and treating the
        corresponding camera frames. The handle is also used to unsubscribe
        from a camera, stopping the background process associated

        Parameters:
            camera_id - The id of the camera to be subscribed
            resolution - CameraResolution object, the resolution of the camera
            fps - The number of frames per second requested for the video
            source (strictly positive float of int)

        Returns:
            handle - The associated camera handle
        """
        try:
            return self.camera_dict[camera_id].subscribe(resolution, fps=fps)

        except KeyError:
            print("This camera does not exist, use a valid camera id")

    def unsubscribeCamera(self, handle):
        """
        Unsubscribe from a camera, using the specified handle

        Parameters:
            handle - The handle retrieved when subscribing to the camera

        Returns:
            success - Boolean, True if unsubscribed successfully, False
            otherwise
        """
        try:
            return Camera._getCameraFromHandle(handle).unsubscribe()

        except KeyError:
            print("Invalid handle, nothing to unsubscribe from")
            return False

    def getCameraFrame(self, handle):
        """
        Returns a frame form the camera associated to the specified handle
        object. Be advised that the subscribeCamera method needs to be called
        beforehand, otherwise a pybullet error will be raised.

        Parameters:
            handle - The handle retrieved when subscribing to the camera

        Returns:
            frame - The current camera frame as a formatted numpy array,
            directly exploitable from OpenCV
        """
        try:
            return Camera._getCameraFromHandle(handle).getFrame()

        except KeyError:
            raise pybullet.error("Invalid handle, cannot retrieve any frame")

    def getCameraResolution(self, handle):
        """
        Returns the resolution of the camera associated to the specified
        handle. Be advised that the subscribeCamera method needs to be called
        beforehand, otherwise a pybullet error will be raised.

        Parameters:
            handle - The handle retrieved when subscribing to the camera

        Returns:
            resolution - a CameraResolution object describing the resolution of
            the robot camera associated to the handle
        """
        try:
            return Camera._getCameraFromHandle(handle).getResolution()

        except KeyError:
            raise pybullet.error("Invalid handle, resolution unavailable")

    def getCameraFps(self, handle):
        """
        Returns the framerate of the camera associated to the specified handle.
        Be advised that the subscribeCamera method needs to be called
        beforehand, otherwise a pybullet error will be raised.

        Parameters:
            handle - the handle retrieved when subscribing to the camera

        Returns:
            fps - The framerate of the camera (frequency of the camera in Hz)
        """
        try:
            return Camera._getCameraFromHandle(handle).getFps()

        except KeyError:
            raise pybullet.error("Invalid handle, framerate unavailable")

    def getCameraLink(self, handle):
        """
        Returns the link of the camera associated to the specified handle. Be
        advised that the subscribeCamera method needs to be called beforehand,
        otherwise a pybullet error will be raised.

        Parameters:
            handle - The handle retrieved when subscribing to the camera

        Returns:
            link - a Link object describing the link to which the robot camera
            associated to the handle is attached
        """
        try:
            return Camera._getCameraFromHandle(handle).getCameraLink()

        except KeyError:
            raise pybullet.error("Invalid handle, cannot retrieve any link")

    def getCamera(self, handle):
        """
        Returns the robot camera associated to the specidied handle. Be advised
        that the subscribeCamera method needs to be called beforehand,
        otherwise a pybullet error will be raised.

        Parameters:
            handle - The handle retrieved when subscribing to the camera

        Returns:
            camera - Camera (CameraRgb or CameraDepth) object, the
            robot camera associated to the handle.
        """
        try:
            return Camera._getCameraFromHandle(handle)

        except KeyError:
            raise pybullet.error("Invalid handle, no associated camera")

    def subscribeImu(self, frequency=None):
        """
        Subscribe to the Inertial Unit of the robot. If the robot doesn't have
        an inertial unit, the method will raise a pybullet error.

        This method starts the background process collecting the Imu data, and
        should be called before trying to access to the Imu data

        Parameters:
            frequency - The desired frequency for the IMU, in Hz. If the
            variable is not specified, the frequency value used when creating
            the Imu object will be used. The value should be specified as an
            int or a float
        """
        try:
            assert self.imu is not None

            if frequency is not None:
                self.imu.setFrequency(frequency)

            self.imu.subscribe()

        except AssertionError:
            raise pybullet.error("No IMU could be found for the virtual robot")

    def unsubscribeImu(self):
        """
        Unsubscribe from the Inertial Unit of the robot. If the robot doesn't
        have an inertial unit, the method will raise a pybullet error.

        This method stops the background process collecting the Imu data, and
        should be called when the Imu data are not useful anymore
        """
        try:
            assert self.imu is not None
            self.imu.unsubscribe()

        except AssertionError:
            raise pybullet.error("No IMU could be found for the virtual robot")

    def getImuGyroscopeValues(self):
        """
        Returns the angular velocity of the IMU in rad/s in the world frame. If
        the robot doesn't have an inertial unit, the method will raise a
        pybullet error.

        One should subscribe to the IMU before calling this method (otherwise,
        the speed will constantly be [0, 0, 0])

        Returns:
            angular_velocity - The angular velocity in rad/s
        """
        if self.imu is not None:
            return self.imu.getGyroscopeValues()
        else:
            raise pybullet.error("No IMU could be found for the virtual robot")

    def getImuAccelerometerValues(self):
        """
        Returns the linear acceleration of the IMU in m/s^2 in the world frame.
        If the robot doesn't have an inertial unit, the method will raise a
        pybullet error.

        One should subscribe to the IMU before calling this method (otherwise,
        the acceleration will constantly be [0, 0, 0])

        Returns:
            linear_acceleration - The linear acceleration in m/s^2
        """
        if self.imu is not None:
            return self.imu.getAccelerometerValues()
        else:
            raise pybullet.error("No IMU could be found for the virtual robot")

    def getImuValues(self):
        """
        Returns the values of the gyroscope and the accelerometer of the IMU
        (angular_velocity, linear_acceleration) in the world frame. If the
        robot doesn't have an inertial unit, the method will raise a pybullet
        error.

        One should subscribe to the IMU before calling this method (otherwise,
        the acceleration and speed will constantly be [0, 0, 0], [0, 0, 0])

        Returns:
            angular_velocity - The angular velocity values in rad/s
            linear_acceleration - The linear acceleration values in m/s^2
        """
        if self.imu is not None:
            return self.imu.getValues()
        else:
            raise pybullet.error("No IMU could be found for the virtual robot")

    def getImu(self):
        """
        Returns the IMU of the robot, as an Imu object. If the robot doesn't
        have an inertial unit, the method will return None

        Returns:
            imu - The IMU of the robot as an Imu object, None if the robot
            doesn't possess an IMU
        """
        return self.imu

    def getFsrValue(self, fsr_name):
        """
        Returns the weight detected on the Z axis of the specified FSR. The
        return value is given in kg (computed from the measured force on the Z
        axis and the gravity of the simulation). If the required fsr does not
        exist, or if no FSR handler has been defined for the robot, the method
        will raise a pybullet error

        WARNING: The returned value is an approximation. Good practice: instead
        of the value itself, take into account the variation of the value, in
        order to detect any change at foot contact level.

        Parameters:
            fsr_name - The name of the FSR, as a string (for instance
            NaoFsr.LFOOT_FL or "LFsrFL_frame")

        Returns:
            fsr_value - The measured value
        """
        if self.fsr_handler is not None:
            return self.fsr_handler.getValue(fsr_name)
        else:
            raise pybullet.error("No FSR handler could be found for the robot")

    def getFsrValues(self, fsr_names):
        """
        Returns all of the FSR weight values for the FSRs corresponding to the
        passed names. If the list of passed names is empty, the method will
        return an empty list. If one of the required FSR does not exist, or if
        no FSR handler has been defined for the robot, the method will raise a
        pybullet error

        Parameters:
            fsr_names - List containing the FSR names (for instance
            NaoFsr.LFOOT)

        Returns:
            fsr_values - The measured values for the corresponding FSRs, as a
            List
        """
        if self.fsr_handler is not None:
            return self.fsr_handler.getValues(fsr_names)
        else:
            raise pybullet.error("No FSR handler could be found for the robot")

    def getTotalFsrValues(self, fsr_names):
        """
        Returns the total weight value (the sum of all FSRs corresponding to
        the names passed to the method). If no names are specified, the method
        will return 0.0. If one of the required FSR does not exist, or if
        no FSR handler has been defined for the robot, the method will raise a
        pybullet error

        Parameters:
            fsr_names - List containing the FSR names (for instance
            NaoFsr.LFOOT)

        Returns:
            total_weight - The sum of all values for the corresponding FSRs
        """
        if self.fsr_handler is not None:
            return self.fsr_handler.getTotalValue(fsr_names)
        else:
            raise pybullet.error("No FSR handler could be found for the robot")

    def getFsrHandler(self):
        """
        Returns the FSR handler of the robot, as a FsrHandler object. If the
        robot doesn't have a FSR handler, the method will return None

        Returns:
            fsr_handler - The FSR handler of the robot as a FsrHandler object,
            None if the robot doesn't possess a FSR handler
        """
        return self.fsr_handler

    def getPosition(self):
        """
        Gets the position of the robot's base in the world frame.

        Returns:
            x - The position of the robot's base on the x axis, in meters
            y - The positions of the robot's base on the y axis in meters
            theta - The rotation of the robot's base on the z axis in meters
        """
        position, quaternions = pybullet.getBasePositionAndOrientation(
            self.robot_model,
            physicsClientId=self.physics_client)

        theta = pybullet.getEulerFromQuaternion(quaternions)[2]
        return position[0], position[1], theta

    def isSelfColliding(self, link_names):
        """
        Specifies if a link is colliding with the rest of the virtual robot.

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
            raise pybullet.error(
                "Unauthorized link checking for self collisions")

    def _setFsrHandler(self, fsr_handler):
        """
        INTERNAL METHOD, To be called by a daughter class. Sets a FsrHandler
        for the robot
        """
        self.fsr_handler = fsr_handler
