#!/usr/bin/env python
# coding: utf-8

from qibullet.robot_module import RobotModule


class Sensor(RobotModule):
    """
    Abstract class describing a virtual sensor
    """

    def __init__(self, robot_model, physics_client, frequency=None):
        """
        Constructor

        Parameters:
            robot_model - The pybullet model of the robot
            physics_client - The if of the simulated instance in which the
            robot is spawned
            frequency - If specified, the acquisition frequency of the sensor,
            in Hz
        """
        RobotModule.__init__(self, robot_model, physics_client)
        self.frequency = None

        self.setFrequency(frequency)

    @property
    def subscribe(self):
        """
        INTERNAL METHOD, Has to be redefined in the daughter classes if the
        sensor uses a subscribing system
        """
        raise NotImplementedError

    @property
    def unsubscribe(self):
        """
        INTERNAL METHOD, Has to be redefined in the daughter classes if the
        sensor uses a subscribing system
        """
        raise NotImplementedError

    def getFrequency(self):
        """
        Returns the frequency of the sensor

        Returns:
            frequency - The frequency in Hz (or None if the frequency is not
            defined)
        """
        return self.frequency

    def setFrequency(self, frequency):
        """
        Sets the frequency of the sensor

        Parameters:
            frequency - The frequency of the sensor in Hz
        """
        if isinstance(frequency, int):
            frequency = float(frequency)

        try:
            assert isinstance(frequency, float)
            assert frequency > 0.0
            self.frequency = frequency

        except AssertionError:
            self.frequency = None
