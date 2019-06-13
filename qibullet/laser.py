#!/usr/bin/env python
# coding: utf-8

import time
import math
import pybullet
import threading
from qibullet.sensor import Sensor

RAY_MISS_COLOR = [0, 1, 0]
RAY_HIT_COLOR = [1, 0, 0]
NUM_RAY = 15
RAY_LENGTH = 3.0  # The theoretical length is 5.6, closer to 3.0 in reality
LASER_ANGLE = 60
LASER_POSITION = [
    [0.0562, 0, -0.334],  # Front laser
    [-0.018, -0.0899, -0.334],  # Right laser
    [-0.018, 0.0899, -0.334]  # Left laser
]
ANGLE_LIST_POSITION = [
    math.radians(LASER_ANGLE/2),  # Front laser
    math.radians(LASER_ANGLE/2) - 1.75728,  # Right laser
    math.radians(LASER_ANGLE/2) + 1.75728  # Left laser
]

NUM_LASER = len(LASER_POSITION)
LASER_FRAMERATE = 6.25


class Laser(Sensor):
    """
    Class representing a virtual laser
    """

    def __init__(
            self,
            robot_model,
            laser_id,
            physicsClientId=0,
            display=False):
        """
        Constructor

        Parameters:
            robot_model - The pybullet model of the robot.
            laser_id - The id of the link (Link type)
            onto which the Lasers' are attached.
            physicsClientId - The id of the simulated instance in which the
            lasers are to be spawned
            display - boolean that allow the display of the laser
        """
        Sensor.__init__(self, robot_model, physicsClientId)
        self.ray_from = []
        self.ray_to = []
        self.ray_ids = []
        self.laser_value = [0] * NUM_RAY * NUM_LASER
        self.laser_id = laser_id
        self.display = display

    @classmethod
    def _getInstances(cls):
        """
        INTERNAL CLASSMETHOD, get all of the Laser instances
        """
        dead = set()

        for ref in cls._instances:
            obj = ref()

            if obj is not None:
                yield obj
            else:
                dead.add(ref)

        cls._instances -= dead

    def isActive(self):
        """
        Check if the lasers are subscribed or not (if the laser thread process
        is active or not)

        Returns:
            boolean - True if the lasers are subscribed, false otherwise
        """
        return self.module_process.isAlive()

    def subscribe(self):
        """
        Subscribe to the laser scan (this will activate the laser scan
        process).
        """
        # No need to subscribe to the laser scan if the lasers are activated
        if self.isActive():
            return

        self._module_termination = False
        self._initializeRays()
        self.module_process = threading.Thread(target=self._laserScan)
        self.module_process.start()

    def unsubscribe(self):
        """
        Unsubscribe from the laser scan (this will deactivate the laser scan
        process)
        """
        if self.isActive():
            self._terminateModule()

    def showLaser(self, display):
        """
        Display debug lines that simulate the laser
        """
        self.display = display

    def getFrontLaserValue(self):
        """
        Return a list of the front laser value (clockwise)
        """
        return self.laser_value[:NUM_RAY]

    def getRightLaserValue(self):
        """
        Return a list of the right laser value (clockwise)
        """
        return self.laser_value[NUM_RAY:2*NUM_RAY]

    def getLeftLaserValue(self):
        """
        Return a list of the left laser value (clockwise)
        """
        return self.laser_value[2*NUM_RAY:]

    def _initializeRays(self):
        """
        INTERNAL METHOD, initialize the laser and all variables needed
        """
        for index in range(NUM_LASER):
            angle = ANGLE_LIST_POSITION[index]
            for i in range(NUM_RAY):
                self.ray_from.append([
                    LASER_POSITION[index][0],
                    LASER_POSITION[index][1],
                    LASER_POSITION[index][2]])
                self.ray_to.append(
                    [LASER_POSITION[index][0] + (RAY_LENGTH) *
                     math.cos(float(i) *
                     math.radians(-LASER_ANGLE)/NUM_RAY + angle),
                     LASER_POSITION[index][1] + (RAY_LENGTH) *
                     math.sin(float(i) *
                     math.radians(-LASER_ANGLE)/NUM_RAY + angle),
                     LASER_POSITION[index][2]])

    def _laserScan(self):
        """
        INTERNAL METHOD, a loop that simulate the laser and update the distance
        value of each laser
        """
        lastLidarTime = time.time()

        while not self._module_termination:
            nowLidarTime = time.time()

            if (nowLidarTime-lastLidarTime > 1/LASER_FRAMERATE):
                results = pybullet.rayTestBatch(
                    self.ray_from,
                    self.ray_to,
                    parentObjectUniqueId=self.robot_model,
                    parentLinkIndex=self.laser_id,
                    physicsClientId=self.physics_client)

                for i in range(NUM_RAY*len(ANGLE_LIST_POSITION)):
                    hitObjectUid = results[i][0]
                    hitFraction = results[i][2]
                    hitPosition = results[i][3]
                    self.laser_value[i] = hitFraction * RAY_LENGTH

                    if self.display:
                        if not self.ray_ids:
                            self._createDebugLine()

                        if (hitFraction == 1.):
                            pybullet.addUserDebugLine(
                                self.ray_from[i],
                                self.ray_to[i],
                                RAY_MISS_COLOR,
                                replaceItemUniqueId=self.ray_ids[i],
                                parentObjectUniqueId=self.robot_model,
                                parentLinkIndex=self.laser_id,
                                physicsClientId=self.physics_client)
                        else:
                            localHitTo = [self.ray_from[i][0]+hitFraction*(
                                        self.ray_to[i][0]-self.ray_from[i][0]),
                                         self.ray_from[i][1]+hitFraction*(
                                        self.ray_to[i][1]-self.ray_from[i][1]),
                                         self.ray_from[i][2]+hitFraction*(
                                        self.ray_to[i][2]-self.ray_from[i][2])]
                            pybullet.addUserDebugLine(
                                self.ray_from[i],
                                localHitTo,
                                RAY_HIT_COLOR,
                                replaceItemUniqueId=self.ray_ids[i],
                                parentObjectUniqueId=self.robot_model,
                                parentLinkIndex=self.laser_id,
                                physicsClientId=self.physics_client)

                    else:
                        if self.ray_ids:
                            self._resetDebugLine()

                lastLidarTime = nowLidarTime

    def _createDebugLine(self):
        """
        INTERNAL METHOD, create all debug lines needed for simulating the
        lasers
        """
        for i in range(NUM_RAY * NUM_LASER):
            self.ray_ids.append(pybullet.addUserDebugLine(
                self.ray_from[i],
                self.ray_to[i],
                RAY_MISS_COLOR,
                parentObjectUniqueId=self.robot_model,
                parentLinkIndex=self.laser_id,
                physicsClientId=self.physics_client))

    def _resetDebugLine(self):
        """
        INTERNAL METHOD, remove all debug lines
        """
        for i in range(len(self.ray_ids)):
            pybullet.removeUserDebugItem(self.ray_ids[i], self.physics_client)

        self.ray_ids = []
