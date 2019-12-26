#!/usr/bin/env python
# coding: utf-8
import sys
import unittest
import pybullet
from qibullet import SimulationManager
from qibullet import NaoVirtual, PepperVirtual, RomeoVirtual
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

        # Test wrong camera ID for subscription
        try:
            CameraTest.robot.subscribeCamera(-3)
            self.assertTrue(True)
        except Exception:
            self.assertTrue(False, "Shouldn't raise an exception")

        # Test wrong camera ID for unsubscription, when active camera is not
        # None
        try:
            CameraTest.robot.subscribeCamera(
                CameraTest.robot.camera_dict.keys()[0])

            CameraTest.robot.unsubscribeCamera(-3)
            self.assertTrue(True)

        except Exception:
            self.assertTrue(False, "Shouldn't raise an exception")
        finally:
            CameraTest.robot.unsubscribeCamera(
                CameraTest.robot.camera_dict.keys()[0])

        # Test subscribing / unsubscribing
        for camera_id, camera_obj in CameraTest.robot.camera_dict.items():
            CameraTest.robot.subscribeCamera(camera_id)
            self.assertEqual(
                Camera.ACTIVE_OBJECT_ID[physics_client],
                id(camera_obj))

            CameraTest.robot.unsubscribeCamera(camera_id)
            self.assertEqual(
                Camera.ACTIVE_OBJECT_ID[physics_client],
                -1)

        # Test camera subscription with invalid resolution
        with self.assertRaises(pybullet.error):
            CameraTest.robot.subscribeCamera(
                CameraTest.robot.camera_dict.keys()[0],
                resolution="invalid")

        # Ensure that if the physics client is not a key of
        # Camera.ACTIVE_OBJECT_ID, the active object id of this instance is set
        # to -1 (need to create a dummy camera to test that)
        dummy_camera = Camera(None, None, None, None, None, physicsClientId=-3)
        self.assertEqual(-1, Camera.ACTIVE_OBJECT_ID[-3])

    def test_get_active_camera(self):
        """
        Test the getActiveCamera method
        """
        for camera_id in CameraTest.robot.camera_dict.keys():
            CameraTest.robot.subscribeCamera(camera_id)

            self.assertEqual(
                camera_id,
                CameraTest.robot.getActiveCamera().getCameraId())

            CameraTest.robot.unsubscribeCamera(camera_id)
            self.assertIsNone(CameraTest.robot.getActiveCamera())

    def test_is_active(self):
        """
        Test the isActive method
        """
        for camera_id, camera in CameraTest.robot.camera_dict.items():
            CameraTest.robot.subscribeCamera(camera_id)
            self.assertTrue(camera.isActive())
            CameraTest.robot.unsubscribeCamera(camera_id)
            self.assertFalse(camera.isActive())

    def test_camera_resolutions(self):
        """
        Test the resolutions for the cameras
        """
        # Test the CameraResolution equality
        self.assertEqual(Camera.K_VGA, Camera.K_VGA)
        self.assertNotEqual(Camera.K_QVGA, Camera.K_QQVGA)

        # Testing that the retrieved camera frames correspond to the required
        # image resolution
        for resolution in [Camera.K_VGA, Camera.K_QVGA, Camera.K_QQVGA]:
            for camera_id in CameraTest.robot.camera_dict.keys():
                CameraTest.robot.subscribeCamera(
                    camera_id,
                    resolution=resolution)

                self.assertEqual(
                    CameraTest.robot.getCameraFrame().shape[1],
                    resolution.width)
                self.assertEqual(
                    CameraTest.robot.getCameraFrame().shape[0],
                    resolution.height)

                self.assertEqual(
                    resolution,
                    CameraTest.robot.getCameraResolution())

                CameraTest.robot.unsubscribeCamera(camera_id)

    def test_camera_channels(self):
        """
        Test the number of channels for each camera.
        """
        for camera_id in CameraTest.robot.camera_dict.keys():
            # If the camera is a depth camera
            if camera_id == PepperVirtual.ID_CAMERA_DEPTH or\
                    camera_id == RomeoVirtual.ID_CAMERA_DEPTH:

                CameraTest.robot.subscribeCamera(
                    camera_id)
                self.assertEqual(
                    len(CameraTest.robot.getCameraFrame().shape),
                    2)

            # if the camera is an RGB camera
            else:
                CameraTest.robot.subscribeCamera(
                    camera_id)
                self.assertEqual(
                    CameraTest.robot.getCameraFrame().shape[2],
                    3)

            CameraTest.robot.unsubscribeCamera(camera_id)

    def test_no_active_camera(self):
        """
        Assert that getCameraFrame and getCameraResolution raise a
        pybullet.error when there is no active camera
        """
        with self.assertRaises(pybullet.error):
            CameraTest.robot.getCameraFrame()

        with self.assertRaises(pybullet.error):
            CameraTest.robot.getCameraResolution()

    def test_camera_link(self):
        """
        Test the getCameraLink method
        """
        # Assert that getCameraLink throws when the active camera is None
        with self.assertRaises(pybullet.error):
            CameraTest.robot.getCameraLink()

        for camera_id, camera_obj in CameraTest.robot.camera_dict.items():
            CameraTest.robot.subscribeCamera(camera_id)

            # Test the getCameraLink method of the Camera class
            self.assertEqual(
                camera_obj.camera_link,
                camera_obj.getCameraLink())

            # Test the getCameraLink method of the RobotVirtual class
            self.assertEqual(
                camera_obj.camera_link,
                CameraTest.robot.getCameraLink())

            CameraTest.robot.unsubscribeCamera(camera_id)

    def test_invalid_fov(self):
        """
        Test the FOV setter of the camera class
        """
        try:
            dummy_camera = Camera(
                None,
                None,
                None,
                "no valid fov",
                ["still not"])

            self.assertTrue(True)

        except Exception:
            self.assertTrue(False, "An invalid FOV should raise an exception")

    def test_get_camera_intrinsics(self):
        """
        Test the getter method for the camera intrinsics
        """
        dummy_camera = Camera(None, None, None, None, None)
        self.assertIsNone(dummy_camera._getCameraIntrinsics())

        for camera_id, camera_obj in CameraTest.robot.camera_dict.items():
            CameraTest.robot.subscribeCamera(camera_id)
            self.assertIsInstance(camera_obj._getCameraIntrinsics(), list)
            CameraTest.robot.unsubscribeCamera(camera_id)


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


class RomeoCameraTest(CameraTest):
    """
    Unittests for Romeo virtual cameras
    """

    @classmethod
    def setUpClass(cls):
        """
        Launches a simulation and spawns the Romeo virtual robot
        """
        CameraTest.simulation = SimulationManager()
        CameraTest.client = CameraTest.simulation.launchSimulation(
            gui=False)

        CameraTest.robot = CameraTest.simulation.spawnRomeo(
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
