#!/usr/bin/env python
# coding: utf-8

import atexit
import weakref
import threading


class RobotModule:
    """
    Abstract class describing a robot module. The Sensor and Controller classes
    inherit that class
    """
    _instances = set()

    def __init__(self, robot_model, physics_client):
        """
        Constructor

        Parameters:
            robot_model - The pybullet model of the robot
            physics_client - The if of the simulated instance in which the
            robot is spawned
        """
        self.robot_model = robot_model
        self.physics_client = physics_client
        self.module_process = threading.Thread(target=None)
        self._instances.add(weakref.ref(self))
        self._module_termination = False
        atexit.register(self._terminateModule)

    def getRobotModel(self):
        """
        Returns the pybullet model to which the module is associated

        Returns:
            robot_model - The pybullet model of the robot
        """
        return self.robot_model

    def getPhysicsClientId(self):
        """
        Returns the id of the simulated instance in which the module is loaded

        Returns:
            physics_client - The id of the simulation in which the robot
            (possessing the module) is spawned
        """
        return self.physics_client

    @classmethod
    def _getInstances(cls):
        """
        INTERNAL CLASSMETHOD, get all of the RobotModule (and daughters)
        instances
        """
        dead = set()

        for ref in cls._instances:
            obj = ref()

            if obj is not None:
                yield obj
            else:
                dead.add(ref)

        cls._instances -= dead

    def _terminateModule(self):
        """
        INTERNAL METHOD, can be called to terminate an asynchronous module
        process. Should only be used when killing the simulation
        """
        self._module_termination = True

        if self.module_process.isAlive():
            self.module_process.join()
