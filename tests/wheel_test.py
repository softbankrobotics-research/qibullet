#!/usr/bin/env python
# coding: utf-8

import sys
import cv2
import time
import math
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

    vel_t = 5
    velt_x = 0
    velt_y = 0
    # vel_r = 1

    while (1):
        keys = p.getKeyboardEvents()
        for k, v in keys.items():
            if (k == p.B3G_RIGHT_ARROW and (v&p.KEY_WAS_TRIGGERED)):
                    velt_y = -vel_t
            if (k == p.B3G_RIGHT_ARROW and (v&p.KEY_WAS_RELEASED)):
                    velt_y = 0
            if (k == p.B3G_LEFT_ARROW and (v&p.KEY_WAS_TRIGGERED)):
                    velt_y = vel_t
            if (k == p.B3G_LEFT_ARROW and (v&p.KEY_WAS_RELEASED)):
                    velt_y = 0

            if (k == p.B3G_UP_ARROW and (v&p.KEY_WAS_TRIGGERED)):
                    velt_x = vel_t
            if (k == p.B3G_UP_ARROW and (v&p.KEY_WAS_RELEASED)):
                    velt_x = 0
            if (k == p.B3G_DOWN_ARROW and (v&p.KEY_WAS_TRIGGERED)):
                    velt_x = -vel_t
            if (k == p.B3G_DOWN_ARROW and (v&p.KEY_WAS_RELEASED)):
                    velt_x = 0

        theta = (math.pi / 3.0)

        vel_fl = -(math.sqrt(3.0)/2.0) * velt_x - 0.5 * velt_y
        vel_b = 0.5 * velt_y
        vel_fr = (math.sqrt(3.0)/2.0) * velt_x - 0.5 * velt_y

        p.setJointMotorControl2(
            pepper.robot_model,
            pepper.joint_dict["WheelFL"].getIndex(),
            p.VELOCITY_CONTROL,
            targetVelocity=-vel_fl,
            force=500)
        p.setJointMotorControl2(
            pepper.robot_model,
            pepper.joint_dict["WheelFR"].getIndex(),
            p.VELOCITY_CONTROL,
            targetVelocity=-vel_fr,
            force=500)
        p.setJointMotorControl2(
            pepper.robot_model,
            pepper.joint_dict["WheelB"].getIndex(),
            p.VELOCITY_CONTROL,
            targetVelocity=-vel_b,
            force=500)
