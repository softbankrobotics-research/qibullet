#!/usr/bin/env python
# coding: utf-8

import sys
IS_VERSION_PYTHON_3 = sys.version_info[0] >= 3


class Joint:
    """
    Class describing a robot joint
    """

    def __init__(self, joint_info):
        """
        Constructor

        Parameters:
            name - The name of the joint
            joint_info - Informations returned by the getJointInfo API
        """
        self.index = joint_info[0]
        self.name = joint_info[1]
        self.lower_limit = joint_info[8]
        self.upper_limit = joint_info[9]
        self.max_effort = joint_info[10]
        self.max_velocity = joint_info[11]

        if IS_VERSION_PYTHON_3:
            self.name = self.name.decode('utf-8')

    def getIndex(self):
        """
        Getter for the index parameter

        Returns:
            index - The joint's index
        """
        return self.index

    def getName(self):
        """
        Getter for the name parameter

        Returns:
            name - The joint's name
        """
        return self.name

    def getLowerLimit(self):
        """
        Getter for the lower limit parameter

        Returns:
            lower_limit - The lower limit of the joint in rad
        """
        return self.lower_limit

    def getUpperLimit(self):
        """
        Getter for the upper limit parameter

        Returns:
            upper_limit - The upper limit of the joint in rad
        """
        return self.upper_limit

    def getMaxEffort(self):
        """
        Getter for the max effort parameter

        Returns:
            max_effort - The max effort for the joint in N.m
        """
        return self.max_effort

    def setMaxEffort(self, max_effort):
        """
        Setter for the max effort parameter

        Parameters:
            max_effort - The max effort for the joint in N.m
        """
        self.max_effort = max_effort

    def getMaxVelocity(self):
        """
        Getter for the max velocity parameter

        Returns:
            max_velocity - The max velocity of the joint in rad/s
        """
        return self.max_velocity

    def setMaxVelocity(self, max_velocity):
        """
        Setter for the max velocity parameter

        Parameters:
            max_velocity - The max velocity for the current joint in rad/s
        """
        self.max_velocity = max_velocity
