#!/usr/bin/env python
# coding: utf-8

import pybullet
from qibullet.sensor import Sensor
from qibullet.helpers import GravityHelper


class NaoFsr:
    """
    Enum for NAO fsrs
    """
    LFOOT_FL = "LFsrFL_frame"
    LFOOT_FR = "LFsrFR_frame"
    LFOOT_RL = "LFsrRL_frame"
    LFOOT_RR = "LFsrRR_frame"
    RFOOT_FL = "LFsrFL_frame"
    RFOOT_FR = "LFsrFR_frame"
    RFOOT_RL = "LFsrRL_frame"
    RFOOT_RR = "LFsrRR_frame"

    LFOOT = [LFOOT_FL, LFOOT_FR, LFOOT_RL, LFOOT_RR]
    RFOOT = [LFOOT_FL, LFOOT_FR, LFOOT_RL, LFOOT_RR]


class Fsr(Sensor):
    """
    Class representing a virtual Force Sensitive Resistor (FSR)
    """

    def __init__(self, robot_model, fsr_link, physicsClientId=0):
        """
        Constructor

        Parameters:
            robot_model - The pybullet model of the robot
            fsr_link - The Link object corresponding to the link the FSR is
            attached to
            physicsClientId - The id of the simulated instance in the
            corresponding robot exists
        """
        Sensor.__init__(
            self,
            robot_model,
            physicsClientId)

        self.fsr_link = fsr_link

    def getValue(self):
        """
        Returns the weight detected on the Z axis of the sensor. The return
        value is given in kg (computed from the measured force on the Z axis
        and the gravity of the simulation)

        WARNING: The returned value is an approximation. Good practice: instead
        of the value itself, take into account the variation of the value, in
        order to detect any change at foot contact level.
        """
        total_force = 0.0
        gravity = GravityHelper.getGravity(self.getPhysicsClientId())[2]

        contact_tuple = pybullet.getContactPoints(
            bodyA=self.getRobotModel(),
            linkIndexA=self.fsr_link.getIndex(),
            physicsClientId=self.getPhysicsClientId())

        for contact in contact_tuple:
            total_force += contact[9]

        return total_force / gravity


class FsrHandler:
    """
    Class defining a tool to handle multiple FSRs on the feet of a bipedal
    robot
    """

    def __init__(self, fsr_dict):
        """
        Constructor. The fsr sensors are passed to the handler through the
        fsr_dict , with the following structure:

        {'fsr_name': Fsr, ...}

        fsr_name being the name of the fsr (string, for instance
        NaoFsr.LFOOT_FL or "LFsrFL_frame"), and Fsr being the associated object

        Parameters:
            fsr_dict - Dict containing the FSRs of the robot
        """
        self.fsr_dict = fsr_dict

    def getValue(self, fsr_name):
        """
        Returns the weight detected on the Z axis of the specified FSR. The
        return value is given in kg (computed from the measured force on the Z
        axis and the gravity of the simulation). If the required FSR does not
        exist, the method will raise a pybullet error

        WARNING: The returned value is an approximation. Good practice: instead
        of the value itself, take into account the variation of the value, in
        order to detect any change at foot contact level.

        Parameters:
            fsr_name - The name of the FSR, as a string (for instance
            NaoFsr.LFOOT_FL or "LFsrFL_frame")

        Returns:
            fsr_value - The measured value
        """
        try:
            return self.fsr_dict[fsr_name].getValue()

        except KeyError:
            raise pybullet.error("The required Fsr does not exist")

    def getValues(self, fsr_names):
        """
        Returns all of the FSR weight values for the FSRs corresponding to the
        passed names. If the list of passed names is empty, the method will
        return an empty list. If one of the required FSRs does not exist, the
        method will raise a pybullet error

        Parameters:
            fsr_names - List containing the FSR names (for instance
            NaoFsr.LFOOT)

        Returns:
            fsr_values - The measured values for the corresponding FSRs, as a
            List
        """
        values = list()

        try:
            for name in fsr_names:
                values.append(self.fsr_dict[name].getValue())

            return values

        except KeyError:
            raise pybullet.error("The required Fsr does not exist")

    def getTotalValue(self, fsr_names):
        """
        Returns the total weight value (the sum of all FSRs corresponding to
        the names passed to the method). If no names are specified, the method
        will return 0.0. If one of the required FSRs does not exist, the
        method will raise a pybullet error

        Parameters:
            fsr_names - List containing the FSR names (for instance
            NaoFsr.LFOOT)

        Returns:
            total_weight - The sum of all values for the corresponding FSRs
        """
        total_weight = sum(self.getValues(fsr_names))

        if isinstance(total_weight, int):
            return float(total_weight)
        else:
            return total_weight
