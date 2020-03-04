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

    def test_subscribe_camera(self):
        """
        Test subscribing to each of Pepper's cameras
        """
        physics_client = CameraTest.client

        # Test wrong camera ID for subscription
        self.assertIsNone(CameraTest.robot.subscribeCamera(-3))

        # Test wrong camera ID for unsubscription, when active camera is not
        # None
        handle = CameraTest.robot.subscribeCamera(
            list(CameraTest.robot.camera_dict.keys())[0])

        self.assertFalse(CameraTest.robot.unsubscribeCamera(-3))
        CameraTest.robot.unsubscribeCamera(handle)

        # Test subscribing / unsubscribing
        for camera_id, camera_obj in CameraTest.robot.camera_dict.items():
            handle = CameraTest.robot.subscribeCamera(camera_id)

            # Check if the provided handle corresponds to the id of the camera
            # object
            self.assertEqual(
                handle,
                id(camera_obj))

            # Check if the camera and the associated handle have been correctly
            # storred in the handles dict
            self.assertIn(handle, Camera._getCameraHandlesDict())

            try:
                self.assertEqual(
                    handle,
                    id(Camera._getCameraFromHandle(handle)))

            except KeyError:
                # Should be able to retrieve the camera associated to the
                # handle without throwing any key error
                self.assertTrue(False)

            self.assertTrue(CameraTest.robot.unsubscribeCamera(handle))
            self.assertNotIn(handle, Camera._getCameraHandlesDict())

        # Test camera subscription with invalid resolution
        with self.assertRaises(pybullet.error):
            CameraTest.robot.subscribeCamera(
                list(CameraTest.robot.camera_dict.keys())[0],
                resolution="invalid")

    def test_get_camera_id(self):
        """
        Test the getCameraId method
        """
        for camera_id in CameraTest.robot.camera_dict.keys():
            handle = CameraTest.robot.subscribeCamera(camera_id)

            # Check the id (PepperVirtual.ID_CAMERA_TOP for instance) of a
            # subscribed camera
            self.assertEqual(
                camera_id,
                CameraTest.robot.getCamera(handle).getCameraId())

            self.assertTrue(CameraTest.robot.unsubscribeCamera(handle))

            with self.assertRaises(pybullet.error):
                CameraTest.robot.getCamera(handle)

    def test_get_camera_resolution(self):
        """
        Test the getCameraResolution method
        """
        # Test the CameraResolution equality
        self.assertEqual(Camera.K_VGA, Camera.K_VGA)
        self.assertNotEqual(Camera.K_QVGA, Camera.K_QQVGA)

        # Testing that the retrieved camera frames correspond to the required
        # image resolution
        for resolution in [Camera.K_VGA, Camera.K_QVGA, Camera.K_QQVGA]:
            for camera_id in CameraTest.robot.camera_dict.keys():
                handle = CameraTest.robot.subscribeCamera(
                    camera_id,
                    resolution=resolution)

                # Check that the camera frame's width and height correspond to
                # the required resolution
                self.assertEqual(
                    CameraTest.robot.getCameraFrame(handle).shape[1],
                    resolution.width)
                self.assertEqual(
                    CameraTest.robot.getCameraFrame(handle).shape[0],
                    resolution.height)

                # Check that the CameraResolution object passed when
                # subscribing corresponds to the resolution of the camera
                self.assertEqual(
                    resolution,
                    CameraTest.robot.getCameraResolution(handle))

                self.assertTrue(CameraTest.robot.unsubscribeCamera(handle))

                with self.assertRaises(pybullet.error):
                    CameraTest.robot.getCameraResolution(handle)

    def test_get_camera_link(self):
        """
        Test the getCameraLink method
        """
        for camera_id, camera_obj in CameraTest.robot.camera_dict.items():
            handle = CameraTest.robot.subscribeCamera(camera_id)

            # Test the getCameraLink method of the Camera class
            self.assertEqual(
                camera_obj.camera_link,
                camera_obj.getCameraLink())

            # Test the getCameraLink method of the RobotVirtual class
            self.assertEqual(
                camera_obj.camera_link,
                CameraTest.robot.getCameraLink(handle))

            self.assertTrue(CameraTest.robot.unsubscribeCamera(handle))

            with self.assertRaises(pybullet.error):
                CameraTest.robot.getCameraLink(handle)

    def test_is_active(self):
        """
        Test the isActive method
        """
        handles = list()

        # Check that the subscribed cameras are active
        for camera_id, camera_obj in CameraTest.robot.camera_dict.items():
            handles.append(CameraTest.robot.subscribeCamera(camera_id))
            self.assertTrue(camera_obj.isActive())

        # Checked that the unsubscribed cameras are inactive
        for handle in handles:
            camera_obj = CameraTest.robot.getCamera(handle)
            self.assertTrue(CameraTest.robot.unsubscribeCamera(handle))
            self.assertFalse(camera_obj.isActive())

    def test_camera_channels(self):
        """
        Test the number of channels for each camera.
        """
        for camera_id in CameraTest.robot.camera_dict.keys():
            if camera_id == PepperVirtual.ID_CAMERA_DEPTH or\
                    camera_id == RomeoVirtual.ID_CAMERA_DEPTH:

                # A depth image should have a shape of 2
                handle = CameraTest.robot.subscribeCamera(camera_id)
                self.assertEqual(
                    len(CameraTest.robot.getCameraFrame(handle).shape),
                    2)

            else:
                # An RGB image should have 3 channels
                handle = CameraTest.robot.subscribeCamera(camera_id)
                self.assertEqual(
                    CameraTest.robot.getCameraFrame(handle).shape[2],
                    3)

            self.assertTrue(CameraTest.robot.unsubscribeCamera(handle))

            with self.assertRaises(pybullet.error):
                CameraTest.robot.getCameraFrame(handle)

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
            self.assertTrue(
                False,
                "An invalid FOV should not raise an exception")

    def test_get_camera_intrinsics(self):
        """
        Test the getter method for the camera intrinsics
        """
        dummy_camera = Camera(None, None, None, None, None)
        self.assertIsNone(dummy_camera._getCameraIntrinsics())

        for camera_id, camera_obj in CameraTest.robot.camera_dict.items():
            handle = CameraTest.robot.subscribeCamera(camera_id)
            self.assertIsInstance(camera_obj._getCameraIntrinsics(), list)
            self.assertTrue(CameraTest.robot.unsubscribeCamera(handle))


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

    def test_subscribe_camera(self):
        CameraTest.test_subscribe_camera(self)

    def test_get_camera_id(self):
        CameraTest.test_get_camera_id(self)

    def test_is_active(self):
        CameraTest.test_is_active(self)

    def test_get_camera_resolution(self):
        CameraTest.test_get_camera_resolution(self)

    def test_camera_channels(self):
        CameraTest.test_camera_channels(self)

    def test_get_camera_link(self):
        CameraTest.test_get_camera_link(self)

    def test_invalid_fov(self):
        CameraTest.test_invalid_fov(self)

    def test_get_camera_intrinsics(self):
        CameraTest.test_get_camera_intrinsics(self)


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

    def test_subscribe_camera(self):
        CameraTest.test_subscribe_camera(self)

    def test_get_camera_id(self):
        CameraTest.test_get_camera_id(self)

    def test_is_active(self):
        CameraTest.test_is_active(self)

    def test_get_camera_resolution(self):
        CameraTest.test_get_camera_resolution(self)

    def test_camera_channels(self):
        CameraTest.test_camera_channels(self)

    def test_get_camera_link(self):
        CameraTest.test_get_camera_link(self)

    def test_invalid_fov(self):
        CameraTest.test_invalid_fov(self)

    def test_get_camera_intrinsics(self):
        CameraTest.test_get_camera_intrinsics(self)


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

    def test_subscribe_camera(self):
        CameraTest.test_subscribe_camera(self)

    def test_get_camera_id(self):
        CameraTest.test_get_camera_id(self)

    def test_is_active(self):
        CameraTest.test_is_active(self)

    def test_get_camera_resolution(self):
        CameraTest.test_get_camera_resolution(self)

    def test_camera_channels(self):
        CameraTest.test_camera_channels(self)

    def test_get_camera_link(self):
        CameraTest.test_get_camera_link(self)

    def test_invalid_fov(self):
        CameraTest.test_invalid_fov(self)

    def test_get_camera_intrinsics(self):
        CameraTest.test_get_camera_intrinsics(self)
