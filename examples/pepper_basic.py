#!/usr/bin/env python
# coding: utf-8

import sys
import cv2
import time
import pybullet as p
import pybullet_data
from qibullet import PepperVirtual

if __name__ == "__main__":
    physicsClient = p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)

    p.setRealTimeSimulation(1)
    p.setGravity(0, 0, -9.81)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadMJCF("mjcf/ground_plane.xml")

    pepper = PepperVirtual()
    pepper.loadRobot([0, 0, 0], [0, 0, 0, 1])

    joint_parameters = list()

    for name, joint in pepper.joint_dict.items():
        if "Finger" not in name and "Thumb" not in name:
            joint_parameters.append((
                p.addUserDebugParameter(name, -4, 4, 0),
                name))

    pepper.subscribeCamera(PepperVirtual.ID_CAMERA_BOTTOM)

    while True:
        img = pepper.getCameraFrame()
        cv2.imshow("bottom camera", img)
        cv2.waitKey(1)

        for joint_parameter in joint_parameters:
            pepper.setAngles(
                joint_parameter[1],
                p.readUserDebugParameter(joint_parameter[0]), 1.0)
