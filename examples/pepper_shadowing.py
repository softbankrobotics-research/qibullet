#!/usr/bin/env python
# coding: utf-8

import qi
import sys
import argparse
from qibullet import SimulationManager


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

    simulation_manager = SimulationManager()
    client = simulation_manager.launchSimulation(gui=True)

    pepper_virtual = simulation_manager.spawnPepper(
        client,
        spawn_ground_plane=True)

    angle_names = list()
    angles_values = list()

    for name in pepper_virtual.joint_dict.keys():
        if "Finger" in name or "Thumb" in name:
            continue
        else:
            angle_names.append(name)

    try:
        while True:
            angles_values = motion.getAngles(angle_names, True)
            pepper_virtual.setAngles(angle_names, angles_values, 1.0)

    except KeyboardInterrupt:
        pass
    finally:
        simulation_manager.stopSimulation(client)
        sys.exit("End the shadowing example")
