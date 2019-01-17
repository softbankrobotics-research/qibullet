#!/usr/bin/env python
# coding: utf-8

import sys
import cv2
import time
import pybullet as p
import pybullet_data

# sys.path.append("../")

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
    forward=0
    turn=0
    while (1):
        keys = p.getKeyboardEvents()
        leftWheelVelocity=0
        rightWheelVelocity=0
        speed=5

        for k,v in keys.items():

            if (k == p.B3G_RIGHT_ARROW and (v&p.KEY_WAS_TRIGGERED)):
                    turn = -0.5
            if (k == p.B3G_RIGHT_ARROW and (v&p.KEY_WAS_RELEASED)):
                    turn = 0
            if (k == p.B3G_LEFT_ARROW and (v&p.KEY_WAS_TRIGGERED)):
                    turn = 0.5
            if (k == p.B3G_LEFT_ARROW and (v&p.KEY_WAS_RELEASED)):
                    turn = 0

            if (k == p.B3G_UP_ARROW and (v&p.KEY_WAS_TRIGGERED)):
                    forward=1
            if (k == p.B3G_UP_ARROW and (v&p.KEY_WAS_RELEASED)):
                    forward=0
            if (k == p.B3G_DOWN_ARROW and (v&p.KEY_WAS_TRIGGERED)):
                    forward=-1
            if (k == p.B3G_DOWN_ARROW and (v&p.KEY_WAS_RELEASED)):
                    forward=0

        rightWheelVelocity+= (forward+turn)*speed
        leftWheelVelocity += (forward-turn)*speed

        p.setJointMotorControl2(pepper.robot_model,pepper.joint_dict["WheelFL"].getIndex(),p.VELOCITY_CONTROL,targetVelocity=leftWheelVelocity,force=500)
        p.setJointMotorControl2(pepper.robot_model,pepper.joint_dict["WheelFR"].getIndex(),p.VELOCITY_CONTROL,targetVelocity=-rightWheelVelocity,force=500)
