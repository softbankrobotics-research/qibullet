#!/usr/bin/env python
# coding: utf-8

import sys
import time
import random
import pybullet as p
import matplotlib.pyplot as plt
from qibullet import SimulationManager


if __name__ == "__main__":
    # We will realize iterations * nb_clients tests
    nb_clients = 10
    iterations = 10

    simulation_manager = SimulationManager()
    client_list = list()
    pepper_list = list()
    angles_list = list()

    for i in range(10):
        client = simulation_manager.launchSimulation(gui=False)
        pepper = simulation_manager.spawnPepper(
            client,
            spawn_ground_plane=True)

        client_list.append(client)
        pepper_list.append(pepper)

    keys = list()
    values = list()
    mean_errors = list()
    collision_links = list()

    for name in pepper_list[0].link_dict.keys():
        if "wrist" in name or "Shoulder" in name or "Head" in name:
            collision_links.append(name)
        # if "Wheel" in name or "Finger" in name or "Thumb" in name:
        #     continue
        # else:
        #     collision_links.append(name)

    for key, value in pepper_list[0].joint_dict.items():
        if "Finger" in key or "Thumb" in key or "Hand" in key:
            continue
        else:
            keys.append(key)
            values.append(value)
            mean_errors.append(0)

    i = 0
    while i < iterations:
        angles_list = list()

        for j in range(nb_clients):
            angles = list()
            for joint in values:
                angles.append(random.uniform(
                    joint.getLowerLimit(),
                    joint.getUpperLimit()))

            print("Angular position " + str(i * (nb_clients) + (j+1)) +
                  "/" + str(nb_clients * iterations))

            angles_list.append(angles)
            pepper_list[j].setAngles(keys, angles, 1.0)

        time.sleep(2)

        try:
            for j in range(nb_clients):
                assert not pepper_list[j].isSelfColliding(collision_links)

        except AssertionError:
            print("Self collision detected")
            continue

        for j in range(nb_clients):
            measured_angles = pepper_list[j].getAnglesPosition(keys)

            for l in range(len(measured_angles)):
                error = abs(angles_list[j][l] - measured_angles[i])
                mean_errors[l] += error

        i += 1

    for i in range(len(mean_errors)):
        mean_errors[i] = mean_errors[i] / (iterations * nb_clients)

    plt.bar(range(len(mean_errors)), mean_errors)
    plt.xticks(
        range(len(mean_errors)),
        tuple(keys),
        rotation='vertical',
        horizontalalignment='left')
    plt.show()

    for client in client_list:
        simulation_manager.stopSimulation(client)
