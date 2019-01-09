#!/usr/bin/env python
# coding: utf-8

import sys
IS_VERSION_PYTHON_3 = sys.version_info[0] >= 3


class Link:
    """
    Class describing a robot link
    """

    def __init__(self, joint_info):
        """
        Constructor

        Parameters:
            name - The name of the joint
            joint_info - Informations returned by the getJointInfo API
        """
        self.index = joint_info[0]
        self.name = joint_info[12]
        self.parent_index = joint_info[16]
        if IS_VERSION_PYTHON_3:
            self.name.decode('utf-8')

    def getIndex(self):
        """
        Getter for the index parameter

        Returns:
            index - The link's index
        """
        return self.index

    def getName(self):
        """
        Getter for the name parameter

        Returns:
            name - The link's name
        """
        return self.name

    def getParentIndex(self):
        """
        Getter for the parent index parameter

        Returns:
            parent_index - The index of the link's parent
        """
        return self.parent_index
