#!/usr/bin/env python
# coding: utf-8


class RobotPosture:
    """
    Class describing a robot posture
    """

    STAND = "Stand"
    STAND_INIT = "StandInit"
    STAND_ZERO = "StandZero"
    CROUCH = "Crouch"

    def __init__(self, posture_name):
        """
        Constructor

        Parameters:
            posture_name - The name of the posture
        """
        self.posture_name = posture_name
        self.joint_names = None
        self.joint_values = None

    def getPostureName(self):
        """
        Returns the posture_name of the posture

        Returns:
            posture_name - The name of the posture
        """
        return self.posture_name

    def getPostureJointNames(self):
        """
        Returns the names of the joints actuated in the posture

        Returns:
            joint_names - List containing the names of the joints
        """
        return self.joint_names

    def getPostureJointValues(self):
        """
        Returns a list of the desired joint positions for the posture

        Returns:
            joint_values - List continaing the desired joint positions
        """
        return self.joint_values

    def isPostureName(self, posture_name):
        """
        Tests if a given string is the name of the posture. The test is not
        case-sensitive

        Parameters:
            posture_name - The name to test

        Returns:
            boolean - True if it is the name of the posture, false otherwise
        """
        return posture_name.lower() == self.getPostureName().lower()

    def _setPosture(self, joint_names, joint_values):
        """
        INTERNAL METHOD, sets the joint names and the joints angles of the
        posture

        Parameters:
            joint_names - List containing the name of the joints to be actuated
            joint_values - List continaing the desired joint positions
        """
        self.joint_names = joint_names
        self.joint_values = joint_values


class PepperPosture(RobotPosture):
    """
    Class describing a robot posture for Pepper
    """

    def __init__(self, posture_name):
        """
        Constructor

        Parameters:
            posture_name - The name of the posture (the joint_names and
            joint_angles will be deduced)
        """
        RobotPosture.__init__(self, posture_name)
        joint_names = [
            "HeadPitch",
            "HeadYaw",
            "HipPitch",
            "HipRoll",
            "KneePitch",
            "LElbowRoll",
            "LElbowYaw",
            "LHand",
            "LShoulderPitch",
            "LShoulderRoll",
            "LWristYaw",
            "RElbowRoll",
            "RElbowYaw",
            "RHand",
            "RShoulderPitch",
            "RShoulderRoll",
            "RWristYaw"]

        if self.isPostureName(RobotPosture.STAND) or\
           self.isPostureName(RobotPosture.STAND_INIT):
            self._setPosture(joint_names, [
                -0.21168947219848633,
                -0.007669925689697266,
                -0.026077747344970703,
                -0.004601955413818359,
                0,
                -0.5184855461120605,
                -1.2179806232452393,
                0.5896309614181519,
                1.5800002813339233,
                0.11658263206481934,
                -0.03072190284729004,
                0.5184855461120605,
                1.225650668144226,
                0.5887521505355835,
                1.5800001621246338,
                -0.11504864692687988,
                0.027570009231567383])

        elif self.isPostureName(RobotPosture.STAND_ZERO):
            self._setPosture(joint_names, [
                0.027611732482910156,
                0.009203910827636719,
                -0.012271881103515625,
                -0.0015339851379394531,
                -0.015339851379394531,
                -0.047553300857543945,
                -0.02914571762084961,
                0.027240753173828125,
                0.02454376220703125,
                0.030679702758789062,
                0.015298128128051758,
                0.03834962844848633,
                0.015339851379394531,
                0.027240753173828125,
                0.047553300857543945,
                -0.03528165817260742,
                -0.013848066329956055])

        elif self.isPostureName(RobotPosture.CROUCH):
            self._setPosture(joint_names, [
                0.6370452046394348,
                0.0,
                -1.0384708642959595,
                -0.0030679702758789062,
                0.5138835906982422,
                -0.008726646192371845,
                -0.48933982849121094,
                0.5949033498764038,
                1.115204095840454,
                0.042951345443725586,
                -0.7808480262756348,
                0.008726646192371845,
                0.4908738136291504,
                0.596660852432251,
                1.1029322147369385,
                -0.018407821655273438,
                0.7869000434875488])
