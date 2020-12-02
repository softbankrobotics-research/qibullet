#!/usr/bin/env python
# coding: utf-8

from qibullet.sensor import Sensor


class Imu(Sensor):
    """
    Class representing a virtual inertial unit
    """

    def __init__(self, robot_model, imu_link, frequency, physics_client=0):
        """
        Constructor

        Parameters:
            robot_model - The pybullet model of the robot
            imu_link - The Link object corresponding to the link the IMU is
            attached to
            frequency - The frequency of the IMU, in Hz
            physics_client - The id of the simulated instance in which the
            IMU should be spawned
        """
        Sensor.__init__(self, robot_model, physics_client)
        self.imu_link = imu_link

    def getGyroscopeValues(self):
        pass

    def getAngleValues(self):
        pass

    def getAccelerometerValues(self):
        pass
