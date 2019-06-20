#!/usr/bin/env python
# coding: utf-8
import sys
import unittest
import pybullet
from qibullet import SimulationManager
from qibullet import NaoVirtual, PepperVirtual
from qibullet import Camera, CameraRgb, CameraDepth, CameraResolution


class CameraTest(unittest.TestCase):
    """
    Unittests for virtual cameras (virtual class, don't use directly)
    """

    def test_camera_subscribe(self):
        """
        Test subscribing to each of Pepper's cameras
        """
        physics_client = CameraTest.client

        CameraTest.robot.subscribeCamera(
            CameraTest.robot.__class__.ID_CAMERA_TOP)
        self.assertEqual(
            Camera.ACTIVE_CAMERA_ID[physics_client],
            id(CameraTest.robot.camera_top))

        CameraTest.robot.unsubscribeCamera(
            CameraTest.robot.__class__.ID_CAMERA_TOP)
        self.assertEqual(
            Camera.ACTIVE_CAMERA_ID[physics_client],
            -1)

        CameraTest.robot.subscribeCamera(
            CameraTest.robot.__class__.ID_CAMERA_BOTTOM)
        self.assertEqual(
            Camera.ACTIVE_CAMERA_ID[physics_client],
            id(CameraTest.robot.camera_bottom))

        CameraTest.robot.unsubscribeCamera(
            CameraTest.robot.__class__.ID_CAMERA_BOTTOM)
        self.assertEqual(
            Camera.ACTIVE_CAMERA_ID[physics_client],
            -1)

        if isinstance(CameraTest.robot, PepperVirtual):
            CameraTest.robot.subscribeCamera(
                CameraTest.robot.__class__.ID_CAMERA_DEPTH)
            self.assertEqual(
                Camera.ACTIVE_CAMERA_ID[physics_client],
                id(CameraTest.robot.camera_depth))

            CameraTest.robot.unsubscribeCamera(
                CameraTest.robot.__class__.ID_CAMERA_DEPTH)
            self.assertEqual(
                Camera.ACTIVE_CAMERA_ID[physics_client],
                -1)

    def test_camera_resolutions(self):
        """
        Test the resolutions for the cameras
        """
        for resolution in [Camera.K_VGA, Camera.K_QVGA, Camera.K_QQVGA]:
            CameraTest.robot.subscribeCamera(
                CameraTest.robot.__class__.ID_CAMERA_TOP,
                resolution=resolution)

            self.assertEqual(
                CameraTest.robot.getCameraFrame().shape[1],
                resolution.width)
            self.assertEqual(
                CameraTest.robot.getCameraFrame().shape[0],
                resolution.height)

    def test_camera_channels(self):
        """
        Test the number of channels for each camera.
        """

        CameraTest.robot.subscribeCamera(
            CameraTest.robot.__class__.ID_CAMERA_TOP)
        self.assertEqual(
            CameraTest.robot.getCameraFrame().shape[2],
            3)

        CameraTest.robot.subscribeCamera(
            CameraTest.robot.__class__.ID_CAMERA_BOTTOM)
        self.assertEqual(
            CameraTest.robot.getCameraFrame().shape[2],
            3)

        if isinstance(CameraTest.robot, PepperVirtual):
            CameraTest.robot.subscribeCamera(
                CameraTest.robot.__class__.ID_CAMERA_DEPTH)
            self.assertEqual(
                len(CameraTest.robot.getCameraFrame().shape),
                2)


class PepperCameraTest(CameraTest):
    """
    Unittests for Pepper virtual cameras
    """

    @classmethod
    def setUpClass(cls):
        """
        Launches a simulation and spawns the Pepper virtual robot
        """
        CameraTest.simulation = SimulationManager()
        CameraTest.client = CameraTest.simulation.launchSimulation(
            gui=False)

        CameraTest.robot = CameraTest.simulation.spawnPepper(
            CameraTest.client,
            spawn_ground_plane=True)

    @classmethod
    def tearDownClass(cls):
        """
        Stops the simulation
        """
        CameraTest.simulation.stopSimulation(
            CameraTest.client)

    def test_camera_subscribe(self):
        CameraTest.test_camera_subscribe(self)

    def test_camera_resolutions(self):
        CameraTest.test_camera_resolutions(self)

    def test_camera_channels(self):
        CameraTest.test_camera_channels(self)


class NaoCameraTest(CameraTest):
    """
    Unittests for Nao virtual cameras
    """

    @classmethod
    def setUpClass(cls):
        """
        Launches a simulation and spawns the NAO virtual robot
        """
        CameraTest.simulation = SimulationManager()
        CameraTest.client = CameraTest.simulation.launchSimulation(
            gui=False)

        CameraTest.robot = CameraTest.simulation.spawnNao(
            CameraTest.client,
            spawn_ground_plane=True)

    @classmethod
    def tearDownClass(cls):
        """
        Stops the simulation
        """
        CameraTest.simulation.stopSimulation(
            CameraTest.client)

    def test_camera_subscribe(self):
        CameraTest.test_camera_subscribe(self)

    def test_camera_resolutions(self):
        CameraTest.test_camera_resolutions(self)

    def test_camera_channels(self):
        CameraTest.test_camera_channels(self)
