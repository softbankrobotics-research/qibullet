#!/usr/bin/env python
# coding: utf-8

import sys
import time
import random
import pybullet as p
import matplotlib.pyplot as plt
from qibullet import SimulationManager


if __name__ == "__main__":
    nb_clients = 2
    iterations = 10

    simulation_manager = SimulationManager()
    client_list = list()
    pepper_list = list()

    for i in range(nb_clients):
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
        if "wrist" in name or "Shoulder" in name:
            collision_links.append(name)

    for key, value in pepper_list[0].joint_dict.items():
        if "Finger" in key or "Thumb" in key or "Hand" in key or "Head" in key:
            continue
        else:
            keys.append(key)
            values.append(value)
            mean_errors.append(0)

    i = 0
    while i < iterations:
        angles_list = [None] * nb_clients

        for j in range(nb_clients):
            angles = list()
            for joint in values:
                angles.append(random.uniform(
                    joint.getLowerLimit(),
                    joint.getUpperLimit()))

            angles_list[j] = angles
            pepper_list[j].setAngles(keys, angles, 1.0)

        time.sleep(2)
        ideal_increase = nb_clients

        if iterations - i < ideal_increase:
            ideal_increase = iterations - i

        increase = ideal_increase
        clean_move_counter = 0

        for j in range(ideal_increase):
            if pepper_list[j].isSelfColliding(collision_links):
                increase -= 1
            else:
                clean_move_counter += 1
                print("Iteration " + str(i + clean_move_counter) + "/" +
                      str(iterations))

                measured_angles = pepper_list[j].getAnglesPosition(keys)

                # print("--------------")
                # print angles_list[j][0]
                # print measured_angles[0]

                for l in range(len(measured_angles)):
                    error = abs(angles_list[j][l] - measured_angles[l])
                    mean_errors[l] += error

        i += increase

    for i in range(len(mean_errors)):
        mean_errors[i] = mean_errors[i] / iterations

    plt.bar(range(len(mean_errors)), mean_errors)
    plt.xticks(
        range(len(mean_errors)),
        tuple(keys),
        rotation='vertical',
        horizontalalignment='left')
    plt.show()

    for client in client_list:
        simulation_manager.stopSimulation(client)
