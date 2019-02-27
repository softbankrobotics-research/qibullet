#!/usr/bin/env python
# coding: utf-8

import atexit
import math
import pybullet
import threading
import time

RAY_MISS_COLOR = [0, 1, 0]
RAY_HIT_COLOR = [1, 0, 0]
NUM_RAY = 16
RAY_LENGTH = 3
RAY_START_LENGTH = 0.25
LASER_ANGLE = 60
LASER_POSITION = [
    [0.0562, 0.0, -0.334],
    [-0.018, 0.0899, -0.334],
    [-0.018, 0.0899, -0.334]
]
# LASER_POSITION = [
#     [0, 0.0, 0.0],
#     [0, 0.0, 0.0],
#     [0, 0.0, 0.0]
# ]
ANGLE_LIST_POSITION = [
    math.radians(LASER_ANGLE),
    math.radians(LASER_ANGLE) + 1.75728,
    math.radians(LASER_ANGLE) - 1.75728]
NUM_LASER = len(LASER_POSITION)
LASER_FRAMERATE = 6.25
PARENTLINKINDEX = 1


class Laser(object):
    """
    Class representing a virtual laser
    """
    def __init__(self, robot_model, base_id, physicsClientId=0):
        self.robot_model = robot_model
        self.physics_client = physicsClientId
        self.laser_thread = threading.Thread()
        self.ray_from = []
        self.ray_to = []
        self.ray_ids = []
        self.base_id = base_id
        self.initLaser()
        self.showLaser()
        self.laser_thread =\
            threading.Thread(target=self.subscribeLaser)
        self.laser_thread.start()
        atexit.register(self._resetActiveLaser)

    def initLaser(self):
        for index in range(NUM_LASER):
            angle = ANGLE_LIST_POSITION[index]
            for i in range(NUM_RAY):
                self.ray_from.append(
                    [(RAY_START_LENGTH + LASER_POSITION[index][0]) *
                     math.sin(float(i) *
                     math.radians(LASER_ANGLE)/NUM_RAY + angle),
                     (RAY_START_LENGTH + LASER_POSITION[index][1]) *
                     math.cos(float(i) *
                     math.radians(LASER_ANGLE)/NUM_RAY + angle),
                     LASER_POSITION[index][2]])
                self.ray_to.append(
                    [(RAY_LENGTH + LASER_POSITION[index][0]) *
                     math.sin(float(i) *
                     math.radians(LASER_ANGLE)/NUM_RAY + angle),
                     (RAY_LENGTH + LASER_POSITION[index][1]) *
                     math.cos(float(i) *
                     math.radians(LASER_ANGLE)/NUM_RAY + angle),
                     LASER_POSITION[index][2]])

    def subscribeLaser(self):
        lastLidarTime = time.time()
        while 1:
            nowLidarTime = time.time()
            if (nowLidarTime-lastLidarTime > 1/LASER_FRAMERATE):
                numThreads = 0
                results = pybullet.rayTestBatch(
                            self.ray_from, self.ray_to, numThreads,
                            parentObjectUniqueId=self.robot_model,
                            parentLinkIndex=self.base_id,
                            physicsClientId=self.physics_client)
                for i in range(NUM_RAY*len(ANGLE_LIST_POSITION)):
                    hitObjectUid = results[i][0]
                    hitFraction = results[i][2]
                    hitPosition = results[i][3]
                    if (hitFraction == 1.):
                        pybullet.addUserDebugLine(
                            self.ray_from[i], self.ray_to[i],
                            RAY_MISS_COLOR,
                            replaceItemUniqueId=self.ray_ids[i],
                            parentObjectUniqueId=self.robot_model,
                            parentLinkIndex=self.base_id,
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
                                   parentLinkIndex=self.base_id,
                                   physicsClientId=self.physics_client)
                lastLidarTime = nowLidarTime

    def showLaser(self):
        for index in range(NUM_LASER):
            angle = ANGLE_LIST_POSITION[index]
            for i in range(NUM_RAY):
                self.ray_ids.append(pybullet.addUserDebugLine(
                    self.ray_from[i], self.ray_to[i],
                    RAY_MISS_COLOR,
                    parentObjectUniqueId=self.robot_model,
                    parentLinkIndex=self.base_id,
                    physicsClientId=self.physics_client))

    def _resetActiveLaser(self):
        """
        INTERNAL METHOD, called when unsubscribing from the active laser, when
        Python is exitted or when the SimulationManager resets/stops a
        simulation instance
        """
        if self.laser_thread.isAlive():
            self.laser_thread.join()
