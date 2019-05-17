#!/usr/bin/env python
# coding: utf-8
import sys
import time
import unittest
import pybullet
import threading
import pybullet_data

from qibullet import SimulationManager
from base_test import PepperBaseTest
from joint_test import PepperJointTest
from laser_test import PepperLaserTest
from camera_test import NaoCameraTest, PepperCameraTest
from posture_test import PepperPostureTest
from self_collision_test import PepperSelfCollisionTest


if __name__ == "__main__":
    simulation_manager = SimulationManager()
    test_loader = unittest.TestLoader()
    test_runner = unittest.TextTestRunner()
    test_results = list()
    has_failed = False

    test_classes = [
        PepperBaseTest,
        PepperJointTest,
        PepperLaserTest,
        PepperCameraTest,
        NaoCameraTest,
        PepperPostureTest,
        PepperSelfCollisionTest]

    physics_client = simulation_manager.launchSimulation(gui=False)

    for test_class in test_classes:
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        pybullet.loadMJCF("mjcf/ground_plane.xml")
        test_results.append(
            test_runner.run(test_loader.loadTestsFromTestCase(test_class)))

        simulation_manager.resetSimulation(physics_client)

    simulation_manager.stopSimulation(physics_client)

    print("------------------------------------------------------------------")
    for i in range(len(test_results)):
        test_count = test_results[i].testsRun
        test_failures = test_results[i].failures
        test_errors = test_results[i].errors
        test_passed = test_count - len(test_failures)

        if len(test_failures) != 0:
            has_failed = True

        print(test_classes[i].__name__ + "[" + str(test_count) + " tests]:")
        print(str(test_passed) + " [\033[1m\033[92mpassed\033[0m]")
        print(str(len(test_failures)) + " [\033[1m\033[91mfailed\033[0m]")

    sys.exit(has_failed)
