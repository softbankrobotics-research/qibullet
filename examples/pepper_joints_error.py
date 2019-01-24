#!/usr/bin/env python
# coding: utf-8

import time
import random
from threading import Thread
import matplotlib.pyplot as plt
from qibullet import SimulationManager


class Task(Thread):
    """
    Threaded task thread
    """

    def __init__(self,
                 pepper,
                 joint_names,
                 collision_links,
                 iterations):
        """
        Constructor

        Parameters:
            pepper - Virtual Pepper model
            joint_names - A list containing the names of the joints to be
            tested
            collision_links - Link tested for self collision
            iterations - Number of iterations for the thread
        """
        Thread.__init__(self)
        self.pepper = pepper
        self.joint_names = joint_names
        self.collision_links = collision_links
        self.iterations = iterations
        self.mean_error = [0] * len(joint_names)

    def run(self):
        """
        Main method loop
        """
        i = 0

        while i < self.iterations:
            angles_list = list()

            for name in self.joint_names:
                angles_list.append(random.uniform(
                    self.pepper.joint_dict[name].getLowerLimit(),
                    self.pepper.joint_dict[name].getUpperLimit()))

            self.pepper.setAngles(self.joint_names, angles_list, 1.0)
            time.sleep(3)

            if self.pepper.isSelfColliding(self.collision_links):
                continue

            measured_angles = self.pepper.getAnglesPosition(self.joint_names)

            for l in range(len(measured_angles)):
                error = abs(angles_list[l] - measured_angles[l])
                self.mean_error[l] += error

            i += 1

    def getResult(self):
        """
        Returns the mean error

        Returns:
            mean_error - The mean error
        """
        return self.mean_error


def main():
    nb_clients = 10
    iterations = 10

    if iterations < nb_clients:
        nb_clients = iterations

    task_list = list()
    iterations_per_task = int(iterations / nb_clients)

    simulation_manager = SimulationManager()
    client_list = list()
    pepper_list = list()
    keys = list()
    collision_links = list()

    gui = False
    for i in range(nb_clients):
        client = simulation_manager.launchSimulation(gui=gui)
        pepper = simulation_manager.spawnPepper(
            client,
            spawn_ground_plane=True)

        if gui:
            gui = False

        client_list.append(client)
        pepper_list.append(pepper)

    collision_links = [
        'r_wrist',
        'LBicep',
        'l_wrist',
        'RBicep',
        'Pelvis']

    for key, value in pepper_list[0].joint_dict.items():
        if "Finger" in key or "Thumb" in key or "Hand" in key or "Head" in key:
            continue
        else:
            keys.append(key)

    for i in range(nb_clients):
        task_list.append(Task(
            pepper_list[i],
            keys,
            collision_links,
            iterations_per_task))

    mean_error = [0] * len(keys)

    for task in task_list:
        task.start()
        print("Task " + str(task.pepper.getPhysicsClientId()) + " started")

    for task in task_list:
        task.join()
        print("Task " + str(task.pepper.getPhysicsClientId()) +
              " finished after " + str(iterations_per_task) + " iteration(s)")

        temp_error = [sum(x) for x in zip(mean_error, task.getResult())]
        mean_error = list(temp_error)

    for client in client_list:
        simulation_manager.stopSimulation(client)

    # Normalize the error given the iteration number
    normalized_mean_error = [x / iterations for x in mean_error]

    plt.bar(range(len(normalized_mean_error)), normalized_mean_error)
    plt.xticks(
        range(len(normalized_mean_error)),
        tuple(keys),
        rotation='vertical',
        horizontalalignment='left')
    plt.show()


if __name__ == "__main__":
    main()
