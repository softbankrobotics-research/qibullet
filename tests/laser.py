#!/usr/bin/env python
# coding: utf-8
import pybullet
from qibullet import SimulationManager
from qibullet import PepperVirtual
import pybullet_data

import time
import math

if __name__ == "__main__":
    simulation_manager = SimulationManager()
    physics_client = simulation_manager.launchSimulation(gui=True)
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
    print pybullet_data.getDataPath()
    pybullet.loadMJCF("mjcf/ground_plane.xml")
    ob = pybullet.loadURDF("cube_small.urdf", [1, 0, 0.10])
    robot = PepperVirtual()
    robot.loadRobot(
        [0, 0, 0],
        [0, 0, 0, 1])
    rayHitColor = [1, 0, 0]
    rayMissColor = [0, 1, 0]
    nowLidarTime = time.time()
    lastLidarTime = time.time()
    lineId = pybullet.addUserDebugLine([0, 0, 0], [0, 0, 1], [1, 0, 0])
    lineId2 = pybullet.addUserDebugLine([0, 0, 0], [0, 0, 1], [1, 0, 0])
    lineId3 = pybullet.addUserDebugLine([0, 0, 0], [0, 0, 1], [1, 0, 0])
    numRays = 16
    rayFrom = []
    rayTo = []
    rayIds = []
    rayLen = 3
    rayStartLen = 0.25
    ANGLE_LASER = 60
    angle_list = [
        math.radians(ANGLE_LASER),
        math.radians(ANGLE_LASER) + math.pi/2,
        math.radians(ANGLE_LASER) - math.pi/2]
    for angle in angle_list:
        for i in range(numRays):
            rayFrom.append(
                [rayStartLen*math.sin(float(i) *
                 math.radians(ANGLE_LASER)/numRays + angle),
                 rayStartLen*math.cos(float(i) *
                 math.radians(ANGLE_LASER)/numRays + angle), 0.03])
            rayTo.append(
                [rayLen*math.sin(float(i) *
                 math.radians(ANGLE_LASER)/numRays + angle),
                 rayLen*math.cos(float(i) *
                 math.radians(ANGLE_LASER)/numRays + angle), 0.03])
            rayIds.append(pybullet.addUserDebugLine(
                rayFrom[i], rayTo[i],
                rayMissColor,
                parentObjectUniqueId=robot.robot_model))
    # lidar at 20Hz
    while 1:
        nowLidarTime = time.time()
        if (nowLidarTime-lastLidarTime > 1/3.26):
            numThreads = 0
            results = pybullet.rayTestBatch(
                        rayFrom, rayTo, numThreads,
                        parentObjectUniqueId=robot.robot_model)
            for i in range(numRays*3):
                hitObjectUid = results[i][0]
                hitFraction = results[i][2]
                hitPosition = results[i][3]
                print("hitFraction[",i,"]=",hitFraction)
                if (hitFraction == 1.):
                    pybullet.addUserDebugLine(
                        rayFrom[i], rayTo[i],
                        rayMissColor,
                        replaceItemUniqueId=rayIds[i],
                        parentObjectUniqueId=robot.robot_model)
                else:
                    localHitTo = [rayFrom[i][0]+hitFraction*(
                                rayTo[i][0]-rayFrom[i][0]),
                                 rayFrom[i][1]+hitFraction*(
                                rayTo[i][1]-rayFrom[i][1]),
                                 rayFrom[i][2]+hitFraction*(
                                rayTo[i][2]-rayFrom[i][2])]
                    pybullet.addUserDebugLine(
                               rayFrom[i],
                               localHitTo,
                               rayHitColor,
                               replaceItemUniqueId=rayIds[i],
                               parentObjectUniqueId=robot.robot_model)
            lastLidarTime = nowLidarTime
    # time.sleep(3)
    simulation_manager.stopSimulation(physics_client)
