#!/usr/bin/env python
# coding: utf-8

import sys
import time
import random
import pybullet as p
import pybullet_data
import matplotlib.pyplot as plt

sys.path.append("../")

from qibullet import PepperVirtual


if __name__ == "__main__":
    physicsClient = p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)

    p.setGravity(0, 0, -9.81)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadMJCF("mjcf/ground_plane.xml")

    a = PepperVirtual()
    a.loadRobot([0, 0, 0], [0, 0, 0, 1])

    p.setRealTimeSimulation(1)

    a.subscribeCamera(PepperVirtual.ID_CAMERA_BOTTOM)

    iterations = 10
    keys = list()
    values = list()
    mean_errors = list()
    collision_links = list()

    for name in a.link_dict.keys():
        if "Wheel" in name or "Finger" in name or "Thumb" in name:
            continue
        else:
            collision_links.append(name)

    for key, value in a.joint_dict.items():
        if "Finger" in key or "Thumb" in key or "Hand" in key:
            continue
        else:
            keys.append(key)
            values.append(value)
            mean_errors.append(0)

    for i in range(iterations):
        angles = list()

        for joint in values:
            angles.append(random.uniform(
                joint.getLowerLimit(),
                joint.getUpperLimit()))

        print("Angular position " + str(i+1) + "/" + str(iterations))
        a.setAngles(keys, angles, 1.0)
        time.sleep(2)

        if a.isSelfColliding(collision_links):
            i -= 1
            continue

        measured_angles = a.getAnglesPosition(keys)

        for i in range(len(measured_angles)):
            error = abs(angles[i] - measured_angles[i])
            mean_errors[i] += error

    for i in range(len(mean_errors)):
        mean_errors[i] = mean_errors[i] / iterations

    plt.bar(range(len(mean_errors)), mean_errors)
    plt.xticks(
        range(len(mean_errors)),
        tuple(keys),
        rotation='vertical',
        horizontalalignment='left')
    plt.show()
