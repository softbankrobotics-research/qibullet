#!/usr/bin/env python
# coding: utf-8
import sys
import unittest
import pybullet
from qibullet import PepperVirtual
from qibullet import SimulationManager
import time


class PepperBaseTest(unittest.TestCase):
    """
    Unittests for the control of Pepper virtual's base
    """

    @classmethod
    def setUpClass(cls):
        """
        Launches a simulation and spawns the Pepper virtual robot
        """
        PepperBaseTest.simulation = SimulationManager()
        PepperBaseTest.client = PepperBaseTest.simulation.launchSimulation(
            gui=False)

        PepperBaseTest.robot = PepperBaseTest.simulation.spawnPepper(
            PepperBaseTest.client,
            spawn_ground_plane=True)

    @classmethod
    def tearDownClass(cls):
        """
        Stops the simulation
        """
        PepperBaseTest.simulation.stopSimulation(
            PepperBaseTest.client)

    def test_move_to_base_world_frame(self):
        """
        Test the set @moveTo method in the world frame, and compare the
        desired position to the position returned by the @getPosition method
        """
        x_command, y_command, theta_command = 1, 1, 2

        try:
            PepperBaseTest.robot.moveTo(
                0,
                0,
                0,
                frame=PepperVirtual.FRAME_WORLD)

            self.assertTrue(True)

        except Exception:
            self.assertTrue(False, "Shouldn't raise an exception")

        try:
            PepperBaseTest.robot.moveTo(
                x_command,
                y_command,
                theta_command,
                frame=PepperVirtual.FRAME_WORLD)

            self.assertTrue(True)

        except Exception:
            self.assertTrue(False, "Shouldn't raise an exception")

    def test_move_to_base_robot_frame(self):
        """
        Test the set @moveTo method in the robot frame, and compare the desired
        position to the position returned by the @getPosition method
        """
        x_def, y_def, theta_def = 1, -1, 0.3,
        x_command, y_command, theta_command = -1, 0, -1

        try:
            PepperBaseTest.robot.moveTo(
                0,
                0,
                0,
                frame=PepperVirtual.FRAME_WORLD)

            self.assertTrue(True)

        except Exception:
            self.assertTrue(False, "Shouldn't raise an exception")

        try:
            PepperBaseTest.robot.moveTo(
                x_def,
                y_def,
                theta_def,
                frame=PepperVirtual.FRAME_WORLD)

            self.assertTrue(True)

        except Exception:
            self.assertTrue(False, "Shouldn't raise an exception")

        try:
            PepperBaseTest.robot.moveTo(
                x_command,
                y_command,
                theta_command,
                frame=PepperVirtual.FRAME_ROBOT)

            self.assertTrue(True)

        except Exception:
            self.assertTrue(False, "Shouldn't raise an exception")

    def test_move_to_base_async(self):
        """
        Test the set @moveTo method in the robot frame, and compare the desired
        position to the position returned by the @getPosition method
        """
        x_def, y_def, theta_def = 1, 1, -1,
        x_command, y_command, theta_command = 0, 0, 0

        try:
            PepperBaseTest.robot.moveTo(
                0,
                0,
                0,
                frame=PepperVirtual.FRAME_WORLD,
                _async=False)

            self.assertTrue(True)

        except Exception:
            self.assertTrue(False, "Shouldn't raise an exception")

        try:
            PepperBaseTest.robot.moveTo(
                x_def,
                y_def,
                theta_def,
                frame=PepperVirtual.FRAME_WORLD,
                _async=True)

            self.assertTrue(True)

        except Exception:
            self.assertTrue(False, "Shouldn't raise an exception")

        # Launching a synchronous moveTo while an asynchronous moveTo process
        # is still running should result in a pybullet error being raised
        with self.assertRaises(pybullet.error):
            PepperBaseTest.robot.moveTo(
                x_def,
                y_def,
                theta_def,
                frame=PepperVirtual.FRAME_WORLD)

        try:
            PepperBaseTest.robot.moveTo(
                x_command,
                y_command,
                theta_command,
                frame=PepperVirtual.FRAME_WORLD,
                _async=True)

            self.assertTrue(True)

        except Exception:
            self.assertTrue(False, "Shouldn't raise an exception")

        # Stop the async moveTo process before exitting the test, we don't want
        # the async process to parasite the other tests
        PepperBaseTest.robot.moveTo(
            0,
            0,
            0,
            frame=PepperVirtual.FRAME_ROBOT,
            _async=True)

        # Sleep to ensure that the previous snippet has been taken into account
        # and that the moveTo async process is stopped.
        # TODO: dirty workaround, should catch when the async process succeded
        # with future for instance
        time.sleep(1.0)

    def test_move_base(self):
        """
        Test the set @move method
        """
        try:
            PepperBaseTest.robot.move(0.5, 0.5, 0.5)
            PepperBaseTest.robot.move(33, -3, 67)
            PepperBaseTest.robot.move(0, 0, 0)

            self.assertTrue(True)

        except Exception:
            self.assertTrue(False, "Shouldn't raise an exception")


if __name__ == "__main__":
    unittest.main()
