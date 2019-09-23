#!/usr/bin/env python
# coding: utf-8

import pybullet


class RobotPosture:
    """
    Class describing a robot posture
    """

    STAND = "Stand"
    STAND_INIT = "StandInit"
    STAND_ZERO = "StandZero"
    CROUCH = "Crouch"
    SIT = "Sit"
    SIT_RELAX = "SitRelax"
    LYING_BELLY = "LyingBelly"
    LYING_BACK = "LyingBack"

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


class NaoPosture(RobotPosture):
    """
    Class describing a robot posture for Nao
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
            "HeadYaw",
            "HeadPitch",
            "LShoulderPitch",
            "LShoulderRoll",
            "LElbowYaw",
            "LElbowRoll",
            "LWristYaw",
            "LHand",
            "LHipYawPitch",
            "LHipRoll",
            "LHipPitch",
            "LKneePitch",
            "LAnklePitch",
            "LAnkleRoll",
            "RHipYawPitch",
            "RHipRoll",
            "RHipPitch",
            "RKneePitch",
            "RAnklePitch",
            "RAnkleRoll",
            "RShoulderPitch",
            "RShoulderRoll",
            "RElbowYaw",
            "RElbowRoll",
            "RWristYaw",
            "RHand"]

        if self.isPostureName(RobotPosture.STAND):
            self._setPosture(joint_names, [
                0.0,
                -0.170000032,
                1.4425844,
                0.224456817,
                -1.20227683,
                -0.417271525,
                0.0999999791,
                0.300000012,
                -0.170009688,
                0.119108438,
                0.12741895,
                -0.0923279151,
                0.0874193981,
                -0.110793173,
                -0.170009688,
                -0.119102433,
                0.127419189,
                -0.0923279151,
                0.0874193311,
                0.110789023,
                1.44258487,
                -0.224456757,
                1.20227683,
                0.417271346,
                0.099999994,
                0.300000012])

        elif self.isPostureName(RobotPosture.STAND_INIT):
            self._setPosture(joint_names, [
                0.0,
                0.0,
                1.4000001,
                0.299999982,
                -1.38999987,
                -1.00999987,
                -1.40129846e-45,
                0.25,
                0.0,
                0.0,
                -0.449999988,
                0.699999988,
                -0.349999994,
                1.13686838e-13,
                0.0,
                0.0,
                -0.449999988,
                0.699999988,
                -0.349999994,
                0.0,
                1.4000001,
                -0.300000042,
                1.38999987,
                1.00999987,
                1.40129846e-45,
                0.25])

        elif self.isPostureName(RobotPosture.STAND_ZERO):
            self._setPosture(joint_names, [
                0.0,
                0.0,
                1.75862134e-35,
                0.00872664712,
                -7.68370234e-36,
                -0.0349065885,
                1.64748683e-36,
                0.0,
                0.0,
                0.0,
                0.0,
                -1.16415322e-10,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                -1.16415322e-10,
                0.0,
                0.0,
                1.75862134e-35,
                -0.00872664712,
                7.68370234e-36,
                0.0349065885,
                -1.64748683e-36,
                0.0])

        elif self.isPostureName(RobotPosture.CROUCH):
            self._setPosture(joint_names, [
                0.0,
                0.100000001,
                1.39685857,
                0.159609884,
                -0.791683376,
                -1.07368624,
                0.149999991,
                0.0,
                -0.25,
                -0.0799999982,
                -0.699999988,
                2.1099999,
                -1.17999995,
                0.0769999996,
                -0.25,
                0.0799999982,
                -0.699999988,
                2.1099999,
                -1.17999995,
                -0.0769999996,
                1.39685857,
                -0.159609884,
                0.791683376,
                1.07368624,
                -0.149999991,
                0.0])

        elif self.isPostureName(RobotPosture.SIT):
            self._setPosture(joint_names, [
                -0.0460619926,
                -0.0399260521,
                0.881872058,
                0.259543777,
                -0.452065587,
                -1.19049561,
                -0.0169160366,
                0.289200008,
                -0.607422113,
                0.273093939,
                -1.53242409,
                1.38976204,
                0.839056015,
                -0.0122299194,
                -0.607422113,
                -0.259204149,
                -1.53588974,
                1.40211821,
                0.848344088,
                0.0230519772,
                0.938332558,
                -0.318750441,
                0.527359486,
                1.23205638,
                0.00609397888,
                0.291599989])

        elif self.isPostureName(RobotPosture.SIT_RELAX):
            self._setPosture(joint_names, [
                -0.0245859623,
                0.0183660984,
                2.08566761,
                0.0887396112,
                -2.08165431,
                -0.199512929,
                -0.949999928,
                0.138399959,
                -0.521517992,
                0.131965876,
                -1.11517596,
                -0.0414600372,
                0.921999991,
                0.0,
                -0.521517992,
                -0.220854044,
                -1.06003618,
                -0.0919981003,
                0.921999991,
                0.0,
                2.08566689,
                -0.154630423,
                2.07549071,
                0.0431277193,
                0.950000048,
                0.139600039])

        elif self.isPostureName(RobotPosture.LYING_BELLY):
            self._setPosture(joint_names, [
                -0.688000023,
                -0.307282031,
                -1.16546321,
                0.133779854,
                -0.166860506,
                -1.08266199,
                -1.59386766,
                0.0124000311,
                -0.0214340687,
                0.0798099041,
                0.299171925,
                -0.0923279151,
                0.742413998,
                -0.0122299194,
                -0.0214340687,
                -0.165630102,
                0.124212027,
                -0.0904641151,
                0.921999991,
                0.0,
                -0.672019422,
                -0.422265947,
                0.706398547,
                1.50837064,
                1.6259979,
                0.171599984])

        elif self.isPostureName(RobotPosture.LYING_BACK):
            self._setPosture(joint_names, [
                -0.00310993195,
                0.153357983,
                1.6777612,
                0.214862406,
                -0.880382538,
                -1.53163767,
                0.246932015,
                0.303200006,
                -0.510780096,
                0.112024069,
                0.375871897,
                -0.0923279151,
                0.863600016,
                -0.0383081436,
                -0.510780096,
                -0.082793951,
                -0.400415897,
                1.73806381,
                0.645855904,
                -0.282214165,
                1.69333243,
                -0.243266225,
                0.879038453,
                1.54461634,
                0.179435983,
                0.360799968])


class RomeoPosture(RobotPosture):
    """
    Class describing a robot posture for Romeo
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
            'LEyePitch',
            'LWristYaw',
            'NeckPitch',
            'LEyeYaw',
            'RWristYaw',
            'LHipYaw',
            'HeadPitch',
            'LShoulderYaw',
            'TrunkYaw',
            'LShoulderPitch',
            'LWristPitch',
            'HeadRoll',
            'RHand',
            'LHand',
            'LKneePitch',
            'RAnkleRoll',
            'RShoulderPitch',
            'RElbowYaw',
            'LHipPitch',
            'RHipPitch',
            'LElbowYaw',
            'RHipYaw',
            'LAnklePitch',
            'REyeYaw',
            'LWristRoll',
            'RShoulderYaw',
            'RWristPitch',
            'LElbowRoll',
            'RWristRoll',
            'LAnkleRoll',
            'RAnklePitch',
            'REyePitch',
            'LHipRoll',
            'RHipRoll',
            'RElbowRoll',
            'RKneePitch',
            'NeckYaw']

        if self.isPostureName(RobotPosture.STAND) or\
           self.isPostureName(RobotPosture.STAND_INIT):
            self._setPosture(joint_names, [
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.749999761581421,
                0.0,
                0.0,
                0.0,
                0.0,
                0.2565000057220459,
                0.0,
                1.749999761581421,
                0.16999998688697815,
                -0.09290000051259995,
                -0.09290000051259995,
                -0.16999998688697815,
                0.0,
                -0.16352200508117676,
                0.0,
                -0.34999996423721313,
                0.0,
                0.0,
                -1.299999713897705,
                0.34999996423721313,
                0.0,
                -0.16352200508117676,
                0.0,
                0.0,
                0.0,
                1.299999713897705,
                0.2565000057220459,
                0.0])

        elif self.isPostureName(RobotPosture.STAND_ZERO):
            self._setPosture(joint_names, [
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.2565000057220459,
                0.0,
                0.0,
                0.0,
                -0.09290000051259995,
                -0.09290000051259995,
                0.0,
                0.0,
                -0.16352200508117676,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                -0.16352200508117676,
                0.0,
                0.0,
                0.0,
                0.0,
                0.2565000057220459,
                0.0])

        elif self.isPostureName(RobotPosture.CROUCH):
            self._setPosture(joint_names, [
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.749999761581421,
                -0.0005399998626671731,
                0.0,
                0.0,
                0.0,
                1.0471899509429932,
                0.0,
                1.749999761581421,
                0.16999998688697815,
                -0.5235980153083801,
                -0.5235980153083801,
                -0.16999998688697815,
                0.0,
                -0.5149000287055969,
                0.0,
                -0.34999996423721313,
                0.0,
                0.0,
                -1.299999713897705,
                0.34999996423721313,
                0.0,
                -0.5149000287055969,
                0.0,
                0.0,
                0.0,
                1.299999713897705,
                1.0471899509429932,
                0.0])
