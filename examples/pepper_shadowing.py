#!/usr/bin/env python
# coding: utf-8

import qi
import sys
import argparse
import pybullet as p
import pybullet_data
from qibullet import PepperVirtual


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    session = qi.Session()
    parser.add_argument(
        "--ip",
        type=str,
        default="127.0.0.1",
        help="The ip address of the robot")

    parser.add_argument(
        "--port",
        type=int,
        default=9559,
        help="The port of the robot")

    args = parser.parse_args()

    try:
        session.connect("tcp://" + args.ip + ":" + str(args.port))
        motion = session.service("ALMotion")

    except RuntimeError:
        sys.exit("Cannot open a qi session")

    physicsClient = p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)

    p.setGravity(0, 0, -9.81)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadMJCF("mjcf/ground_plane.xml")

    pepper_virtual = PepperVirtual()
    pepper_virtual.loadRobot([0, 0, 0], [0, 0, 0, 1])

    angle_names = list()
    angles_values = list()

    for name in pepper_virtual.joint_dict.keys():
        if "Finger" in name or "Thumb" in name:
            continue
        else:
            angle_names.append(name)

    p.setRealTimeSimulation(1)

    try:
        while True:
            angles_values = motion.getAngles(angle_names, True)
            pepper_virtual.setAngles(angle_names, angles_values, 1.0)

    except KeyboardInterrupt:
        sys.exit("End the shadowing example")
