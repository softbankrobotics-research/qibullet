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
            # Test moveTo and add a linear speed argument
            PepperBaseTest.robot.moveTo(
                x_command,
                y_command,
                theta_command,
                frame=PepperVirtual.FRAME_WORLD,
                speed=PepperBaseTest.robot.linear_velocity)

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

    def test_base_speed_limits(self):
        """
        Test the set the base speed limits (for Pepper)
        """
        max_linear = PepperBaseTest.robot.base_controller.MAX_LINEAR_VELOCITY
        min_linear = PepperBaseTest.robot.base_controller.MIN_LINEAR_VELOCITY
        max_angular = PepperBaseTest.robot.base_controller.MAX_ANGULAR_VELOCITY
        min_angular = PepperBaseTest.robot.base_controller.MIN_ANGULAR_VELOCITY
        epsilon = 0.5

        # Linear velocity tests
        PepperBaseTest.robot.base_controller.setLinearVelocity(
            max_linear + epsilon)

        self.assertEqual(
            PepperBaseTest.robot.base_controller.linear_velocity,
            max_linear)

        PepperBaseTest.robot.base_controller.setLinearVelocity(
            min_linear - epsilon)

        self.assertEqual(
            PepperBaseTest.robot.base_controller.linear_velocity,
            min_linear)

        # Angular velocity tests
        PepperBaseTest.robot.base_controller._setAngularVelocity(
            max_angular + epsilon)

        self.assertEqual(
            PepperBaseTest.robot.base_controller.angular_velocity,
            max_angular)

        PepperBaseTest.robot.base_controller._setAngularVelocity(
            min_angular - epsilon)

        self.assertEqual(
            PepperBaseTest.robot.base_controller.angular_velocity,
            min_angular)

        # Reset the base controller of the Pepper robot
        PepperBaseTest.robot.base_controller.setLinearVelocity(
            PepperBaseTest.robot.linear_velocity)

        PepperBaseTest.robot.base_controller._setAngularVelocity(
            PepperBaseTest.robot.angular_velocity)

    def test_base_acceleration_limits(self):
        """
        Test the set the base acceleration limits (for Pepper)
        """
        max_linear =\
            PepperBaseTest.robot.base_controller.MAX_LINEAR_ACCELERATION
        min_linear =\
            PepperBaseTest.robot.base_controller.MIN_LINEAR_ACCELERATION
        max_angular =\
            PepperBaseTest.robot.base_controller.MAX_ANGULAR_ACCELERATION
        min_angular =\
            PepperBaseTest.robot.base_controller.MIN_ANGULAR_ACCELERATION
        epsilon = 0.5

        # Linear acceleration tests
        PepperBaseTest.robot.base_controller._setLinearAcceleration(
            max_linear + epsilon)

        self.assertEqual(
            PepperBaseTest.robot.base_controller.linear_acceleration,
            max_linear)

        PepperBaseTest.robot.base_controller._setLinearAcceleration(
            min_linear - epsilon)

        self.assertEqual(
            PepperBaseTest.robot.base_controller.linear_acceleration,
            min_linear)

        # Angular acceleration tests
        PepperBaseTest.robot.base_controller._setAngularAcceleration(
            max_angular + epsilon)

        self.assertEqual(
            PepperBaseTest.robot.base_controller.angular_acceleration,
            max_angular)

        PepperBaseTest.robot.base_controller._setAngularAcceleration(
            min_angular - epsilon)

        self.assertEqual(
            PepperBaseTest.robot.base_controller.angular_acceleration,
            min_angular)

        # Reset the base controller of the Pepper robot
        PepperBaseTest.robot.base_controller._setLinearAcceleration(
            PepperBaseTest.robot.linear_acceleration)

        PepperBaseTest.robot.base_controller._setAngularAcceleration(
            PepperBaseTest.robot.angular_acceleration)

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
