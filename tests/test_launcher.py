#!/usr/bin/env python
# coding: utf-8
import os
import sys
import unittest


if __name__ == "__main__":
    # Add the file's parent folder and current folder (for the coverage)to the
    # path
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)) + '/..')
    sys.path.insert(0, '')

    # Create unittest objects
    test_loader = unittest.TestLoader()
    test_runner = unittest.TextTestRunner()
    test_results = list()
    tests_failed = False

    # Import the unittests
    try:
        from base_test import PepperBaseTest
        from link_test import NaoLinkTest
        from link_test import RomeoLinkTest
        from link_test import PepperLinkTest
        from joint_test import NaoJointTest
        from joint_test import RomeoJointTest
        from joint_test import PepperJointTest
        from laser_test import PepperLaserTest
        from camera_test import NaoCameraTest
        from camera_test import RomeoCameraTest
        from camera_test import PepperCameraTest
        from posture_test import NaoPostureTest
        from posture_test import PepperPostureTest
        from posture_test import RomeoPostureTest
        from virtual_robot_test import VirtualRobotTest
        from self_collision_test import NaoSelfCollisionTest
        from self_collision_test import PepperSelfCollisionTest
        from simulation_manager_test import SimulationManagerTest

    except ImportError as e:
        print(str(e))
        sys.exit(1)

    test_classes = [
        SimulationManagerTest,
        VirtualRobotTest,
        PepperBaseTest,
        NaoLinkTest,
        RomeoLinkTest,
        PepperLinkTest,
        NaoJointTest,
        RomeoJointTest,
        PepperJointTest,
        PepperLaserTest,
        NaoCameraTest,
        RomeoCameraTest,
        PepperCameraTest,
        NaoPostureTest,
        PepperPostureTest,
        RomeoPostureTest,
        NaoSelfCollisionTest,
        PepperSelfCollisionTest]

    for test_class in test_classes:
        test_results.append(
            test_runner.run(test_loader.loadTestsFromTestCase(test_class)))

    total_count = 0
    total_fail = 0
    total_success = 0
    print("------------------------------------------------------------------")
    for i in range(len(test_results)):
        test_count = test_results[i].testsRun
        test_failures = test_results[i].failures
        test_errors = test_results[i].errors
        test_passed = test_count - len(test_failures)

        total_count += test_count
        total_fail += len(test_failures)
        total_success += test_passed

        if len(test_failures) != 0:
            tests_failed = True

        print(test_classes[i].__name__ + "[" + str(test_count) + " tests]:")
        print(str(test_passed) + " [\033[1m\033[92mpassed\033[0m]")
        print(str(len(test_failures)) + " [\033[1m\033[91mfailed\033[0m]")

    print("------------------------------------------------------------------")
    print("\033[1m\33[33mSummary: " + str(total_count) + " tests\033[0m")
    print(str(total_success) + " [\033[1m\033[92mpassed\033[0m]")
    print(str(total_fail) + " [\033[1m\033[91mfailed\033[0m]")

    sys.exit(tests_failed)
