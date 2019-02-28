#!/usr/bin/env python
# coding: utf-8

import atexit
import math
import pybullet
import threading
import time

RAY_MISS_COLOR = [0, 1, 0]
RAY_HIT_COLOR = [1, 0, 0]
NUM_RAY = 15
RAY_LENGTH = 5.6
RAY_START_LENGTH = 0.1
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


class Laser(object):
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
        self.robot_model = robot_model
        self.physics_client = physicsClientId
        self.laser_thread = threading.Thread()
        self.ray_from = []
        self.ray_to = []
        self.ray_ids = []
        self.laser_value = [0] * NUM_RAY * NUM_LASER
        self.laser_id = laser_id
        self.display = display
        self._initLaser()
        self.laser_thread =\
            threading.Thread(target=self._subscribeLaser)
        self.laser_thread.start()
        atexit.register(self._resetActiveLaser)

    def showLaser(self):
        """
        Display debug lines that simulate the laser
        """
        self.display = True

    def getFrontLaserValue(self):
        """
        Return a list of the front laser value
        """
        return self.laser_value[:NUM_RAY]

    def getRightLaserValue(self):
        """
        Return a list of the right laser value
        """
        return self.laser_value[NUM_RAY:2*NUM_RAY]

    def getLeftLaserValue(self):
        """
        Return a list of the left laser value
        """
        return self.laser_value[2*NUM_RAY:]

    def _initLaser(self):
        """
        INTERNAL METHOD, initialize the laser and all variables needed
        """
        for index in range(NUM_LASER):
            angle = ANGLE_LIST_POSITION[index]
            for i in range(NUM_RAY):
                self.ray_from.append(
                    [LASER_POSITION[index][0] + (RAY_START_LENGTH) *
                     math.cos(float(i) *
                     math.radians(-LASER_ANGLE)/NUM_RAY + angle),
                     LASER_POSITION[index][1] + (RAY_START_LENGTH) *
                     math.sin(float(i) *
                     math.radians(-LASER_ANGLE)/NUM_RAY + angle),
                     LASER_POSITION[index][2]])
                self.ray_to.append(
                    [LASER_POSITION[index][0] + (RAY_LENGTH) *
                     math.cos(float(i) *
                     math.radians(-LASER_ANGLE)/NUM_RAY + angle),
                     LASER_POSITION[index][1] + (RAY_LENGTH) *
                     math.sin(float(i) *
                     math.radians(-LASER_ANGLE)/NUM_RAY + angle),
                     LASER_POSITION[index][2]])

    def _subscribeLaser(self):
        """
        INTERNAL METHOD, a loop that simulate the laser and update the distance
        value of each laser
        """
        lastLidarTime = time.time()
        while 1:
            nowLidarTime = time.time()
            if (nowLidarTime-lastLidarTime > 1/LASER_FRAMERATE):
                numThreads = 0
                results = pybullet.rayTestBatch(
                            self.ray_from, self.ray_to, numThreads,
                            parentObjectUniqueId=self.robot_model,
                            parentLinkIndex=self.laser_id,
                            physicsClientId=self.physics_client)
                print self.getRightLaserValue()
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
                                self.ray_from[i], self.ray_to[i],
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
                lastLidarTime = nowLidarTime

    def _createDebugLine(self):
        """
        INTERNAL METHOD, create all debug lines needed for simulating the
        lasers
        """
        for i in range(NUM_RAY * NUM_LASER):
            self.ray_ids.append(pybullet.addUserDebugLine(
                self.ray_from[i], self.ray_to[i],
                RAY_MISS_COLOR,
                parentObjectUniqueId=self.robot_model,
                parentLinkIndex=self.laser_id,
                physicsClientId=self.physics_client))

    def _resetActiveLaser(self):
        """
        INTERNAL METHOD, called when unsubscribing from the active laser, when
        Python is exitted or when the SimulationManager resets/stops a
        simulation instance
        """
        if self.laser_thread.isAlive():
            self.laser_thread.join()
