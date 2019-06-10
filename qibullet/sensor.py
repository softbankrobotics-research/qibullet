#!/usr/bin/env python
# coding: utf-8

from qibullet.robot_module import RobotModule


class Sensor(RobotModule):
    """
    Abstract class describing a virtual sensor
    """

    def __init__(self, robot_model, physics_client):
        """
        Constructor

        Parameters:
            robot_model - The pybullet model of the robot
            physics_client - The if of the simulated instance in which the
            robot is spawned
        """
        RobotModule.__init__(self, robot_model, physics_client)
