#!/usr/bin/env python
# coding: utf-8
import sys
import unittest
import pybullet
from qibullet import PepperVirtual
from qibullet import Camera, CameraRgb, CameraDepth, CameraResolution


class PepperCameraTest(unittest.TestCase):
    """
    Unittests for Pepper virtual's cameras
    """

    @classmethod
    def setUpClass(cls):
        """
        Launches a simulation and spawns the Pepper virtual robot
        """
        cls.pepper_virtual = PepperVirtual()
        cls.pepper_virtual.loadRobot(
            [0, 0, 0],
            [0, 0, 0, 1])

    def test_camera_subscribe(self):
        """
        Test subscribing to each of Pepper's cameras
        """
        physics_client = PepperCameraTest.pepper_virtual.physics_client

        PepperCameraTest.pepper_virtual.subscribeCamera(
            PepperVirtual.ID_CAMERA_TOP)
        self.assertEqual(
            Camera.ACTIVE_CAMERA_ID[physics_client],
            id(PepperCameraTest.pepper_virtual.camera_top))
        PepperCameraTest.pepper_virtual.unsubscribeCamera(
            PepperVirtual.ID_CAMERA_TOP)
        self.assertEqual(
            Camera.ACTIVE_CAMERA_ID[physics_client],
            -1)

        PepperCameraTest.pepper_virtual.subscribeCamera(
            PepperVirtual.ID_CAMERA_BOTTOM)
        self.assertEqual(
            Camera.ACTIVE_CAMERA_ID[physics_client],
            id(PepperCameraTest.pepper_virtual.camera_bottom))
        PepperCameraTest.pepper_virtual.unsubscribeCamera(
            PepperVirtual.ID_CAMERA_BOTTOM)
        self.assertEqual(
            Camera.ACTIVE_CAMERA_ID[physics_client],
            -1)

        PepperCameraTest.pepper_virtual.subscribeCamera(
            PepperVirtual.ID_CAMERA_DEPTH)
        self.assertEqual(
            Camera.ACTIVE_CAMERA_ID[physics_client],
            id(PepperCameraTest.pepper_virtual.camera_depth))
        PepperCameraTest.pepper_virtual.unsubscribeCamera(
            PepperVirtual.ID_CAMERA_DEPTH)
        self.assertEqual(
            Camera.ACTIVE_CAMERA_ID[physics_client],
            -1)

    def test_camera_resolutions(self):
        """
        Test the resolutions for the cameras
        """
        for resolution in [Camera.K_VGA, Camera.K_QVGA, Camera.K_QQVGA]:
            PepperCameraTest.pepper_virtual.subscribeCamera(
                PepperVirtual.ID_CAMERA_TOP,
                resolution=resolution)

            self.assertEqual(
                PepperCameraTest.pepper_virtual.getCameraFrame().shape[1],
                resolution.width)
            self.assertEqual(
                PepperCameraTest.pepper_virtual.getCameraFrame().shape[0],
                resolution.height)

    def test_camera_channels(self):
        """
        Test the number of channels for each camera.
        """

        PepperCameraTest.pepper_virtual.subscribeCamera(
            PepperVirtual.ID_CAMERA_TOP)
        self.assertEqual(
            PepperCameraTest.pepper_virtual.getCameraFrame().shape[2],
            3)

        PepperCameraTest.pepper_virtual.subscribeCamera(
            PepperVirtual.ID_CAMERA_BOTTOM)
        self.assertEqual(
            PepperCameraTest.pepper_virtual.getCameraFrame().shape[2],
            3)

        PepperCameraTest.pepper_virtual.subscribeCamera(
            PepperVirtual.ID_CAMERA_DEPTH)
        self.assertEqual(
            len(PepperCameraTest.pepper_virtual.getCameraFrame().shape),
            2)
