#!/usr/bin/env python
# coding: utf-8

import sys
import pybullet
from qibullet.link import Link
from qibullet.joint import Joint

IS_VERSION_PYTHON_3 = sys.version_info[0] >= 3


class RobotVirtual:
    """
    Mother class representing a virtual robot
    """

    def __init__(self, description_file):
        """
        Constructor

        Parameters:
            - description_file: The file giving the description of the virtual
            robot. For now, only URDF is handled
        """
        self.description_file = description_file
        self.physics_client = 0
        self.joint_dict = dict()
        self.link_dict = dict()

    def loadRobot(self, translation, quaternion, physicsClientId=0):
        """
        Loads the robot into a simulation, loads the joints and the links
        descriptions. The joints are set to 0 rad

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
            print("Cannot load robot model: " + str(e))
            return False

        for i in range(pybullet.getNumJoints(
                self.robot_model,
                physicsClientId=self.physics_client)):
            if IS_VERSION_PYTHON_3:
                # PYTHON 3 version needs a conversion bytes to str
                joint_info = pybullet.getJointInfo(
                    self.robot_model,
                    i,
                    physicsClientId=self.physics_client)
                self.link_dict[joint_info[12].decode('utf-8')] =\
                    Link(joint_info)

                if joint_info[2] == pybullet.JOINT_PRISMATIC or\
                        joint_info[2] == pybullet.JOINT_REVOLUTE:
                    self.joint_dict[joint_info[1].decode('utf-8')] =\
                        Joint(joint_info)
            else:
                # PYTHON 2 Version
                joint_info = pybullet.getJointInfo(
                    self.robot_model,
                    i,
                    physicsClientId=self.physics_client)

                self.link_dict[joint_info[12]] = Link(joint_info)

                if joint_info[2] == pybullet.JOINT_PRISMATIC or\
                        joint_info[2] == pybullet.JOINT_REVOLUTE:
                    self.joint_dict[joint_info[1]] = Joint(joint_info)

            pybullet.setJointMotorControl2(
                self.robot_model,
                i,
                pybullet.POSITION_CONTROL,
                0,
                physicsClientId=self.physics_client)

        return True

    def getPhysicsClientId(self):
        """
        Gets the id of the id of the simulation in which this robot instance is
        loaded

        Returns:
            physics_client - The id of the simulation
        """
        return self.physics_client

    def setAngles(self, joint_names, joint_values, percentage_speed):
        """
        Set angles on the robot's joints. Tests have to be performed by the
        child class to guarantee the validity of the input parameters

        Parameters:
            joint_names - List of string containing the name of the joints
            to be controlled
            joint_values - List of values corresponding to the angles in
            radians to be applied
            percentage_speed - Percentage of the max speed to be used for the
            movement, has to be strictly superior to 0 and inferior or equal to
            1
        """
        try:
            assert len(joint_names) == len(joint_values)
            assert percentage_speed > 0.0 and percentage_speed <= 1.0

        except AssertionError:
            print("Error in the setAngles parameters")
            return

        joint_speed = 0

        for joint_name, joint_value in zip(joint_names, joint_values):
            joint_speed =\
                self.joint_dict[joint_name].getMaxVelocity() *\
                percentage_speed

            pybullet.setJointMotorControl2(
                self.robot_model,
                self.joint_dict[joint_name].getIndex(),
                pybullet.POSITION_CONTROL,
                targetPosition=joint_value,
                maxVelocity=joint_speed,
                physicsClientId=self.physics_client)

    def getAnglesPosition(self, joint_names):
        """
        Gets the position of the robot's joints in radians. If one of the joint
        doesn't exist, the method will raise a KeyError

        Parameters:
            joint_names - List of string containing the names of the joints

        Returns:
            joint_positions - List of floats containing the joint's positions
        """
        joint_positions = list()

        for joint_name in joint_names:
            joint_positions.append(pybullet.getJointState(
                self.robot_model,
                self.joint_dict[joint_name].getIndex(),
                physicsClientId=self.physics_client)[0])

        return joint_positions

    def getPosition(self):
        """
        Gets the position of the robot's base in the world frame

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
