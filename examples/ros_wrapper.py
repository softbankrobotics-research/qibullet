#!/usr/bin/env python
# coding: utf-8

import sys
import rospy
import pybullet as p
import pybullet_data

sys.path.append("../")

from qibullet import PepperVirtual
from qibullet import PepperRosWrapper

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

    wrap = PepperRosWrapper()
    wrap.launchWrapper(pepper, "/naoqi_driver")

    pepper.subscribeCamera(PepperVirtual.ID_CAMERA_TOP)
    rospy.spin()
