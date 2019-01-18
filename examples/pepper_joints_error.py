#!/usr/bin/env python
# coding: utf-8

import sys
import time
import random
import pybullet as p
import matplotlib.pyplot as plt
from qibullet import SimulationManager


if __name__ == "__main__":
    nb_clients = 10
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
        ideal_increase = nb_clients

        if ideal_increase > iterations - i:
            ideal_increase = iterations - i

        for j in range(ideal_increase):
            angles_list[j] = [0] * len(values)
            for l in range(len(values)):
                angles_list[j][l] = random.uniform(
                    values[l].getLowerLimit(),
                    values[l].getUpperLimit())

            pepper_list[j].setAngles(keys, angles_list[j], 1.0)

        time.sleep(2)

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

                for l in range(len(measured_angles)):
                    error = abs(angles_list[j][l] - measured_angles[l])
                    mean_errors[l] += error

        i += increase

    for client in client_list:
        simulation_manager.stopSimulation(client)

    for m in range(len(mean_errors)):
        mean_errors[m] = mean_errors[m] / iterations

    plt.bar(range(len(mean_errors)), mean_errors)
    plt.xticks(
        range(len(mean_errors)),
        tuple(keys),
        rotation='vertical',
        horizontalalignment='left')
    plt.show()
