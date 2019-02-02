#!/usr/bin/env python
# coding: utf-8

import atexit
import weakref
import pybullet
import threading
import numpy as np
from qibullet.link import Link


class CameraResolution:
    """
    Enumeration of the camera resolutions
    """

    def __init__(self, width, height):
        """
        Constructor

        Parameters:
            width - Width resolution in pixels
            height - Height resolution in pixels
        """
        self.width = width
        self.height = height

    def __eq__(self, resolution):
        """
        Overloading the equal operator

        Parameters:
            resolution - the comparing resolution
        """
        if self.width == resolution.width and self.height == resolution.height:
            return True
        else:
            return False


class Camera:
    """
    Class representing a virtual camera
    """
    _instances = set()
    ACTIVE_CAMERA_ID = dict()

    K_QQVGA = CameraResolution(160, 120)
    K_QVGA = CameraResolution(320, 240)
    K_VGA = CameraResolution(640, 480)
    K_QQ720p = CameraResolution(320, 180)
    K_Q720p = CameraResolution(640, 360)
    K_720p = CameraResolution(1280, 720)

    def __init__(
            self,
            robot_model,
            link,
            physicsClientId=0,
            near_plane=0.01,
            far_plane=100):
        """
        Constructor

        Parameters:
            robot_model - The pybullet model of the robot
            link - The link (Link type) onto which the camera's attached
            physicsClientId - The id of the simulated instance in which the
            camera is to be spawned
            near_plane - The near plane distance
            far_plane - The far plane distance
        """
        self.robot_model = robot_model
        self.physics_client = physicsClientId
        self.link = link
        self.near_plane = near_plane
        self.far_plane = far_plane
        self.frame = None
        self.projection_matrix = None
        self.resolution = None
        self.fov = None
        self.extraction_thread = threading.Thread()
        self.frame_lock = threading.Lock()
        self.resolution_lock = threading.Lock()
        self._instances.add(weakref.ref(self))
        self._resetActiveCamera()
        atexit.register(self._resetActiveCamera)

    @classmethod
    def _getInstances(cls):
        """
        INTERNAL CLASSMETHOD, get all of the Camera (and daughters) instances
        """
        dead = set()

        for ref in cls._instances:
            obj = ref()

            if obj is not None:
                yield obj
            else:
                dead.add(ref)

        cls._instances -= dead

    def subscribe(self, id, resolution):
        """
        Subscribing method for the camera. The FOV has to be specified
        beforehand

        Parameters:
            id - The id of the Python object calling the method
            resolution - CameraResolution object, the resolution of the camera
        """
        # Lets the extracting thread die before launching another one if the
        # same camera calls the subscribing method for a second time
        self._resetActiveCamera()
        self._setResolution(resolution)
        Camera.ACTIVE_CAMERA_ID[self.physics_client] = id

    def unsubscribe(self, id):
        """
        Method stopping the frame retreival thread for a camera
        """
        if Camera.ACTIVE_CAMERA_ID[self.physics_client] == id:
            self._resetActiveCamera()

    def getFrame(self):
        """
        Returns the current frame

        Returns:
            frame - The current frame
        """
        with self.frame_lock:
            return self.frame

    def isActive(self):
        """
        Specifies if the camera is active or not

        Returns:
            is_active - Boolean, True if the camera is subscribed to, False
            otherwise
        """
        return id(self) == Camera.ACTIVE_CAMERA_ID[self.physics_client]

    def getResolution(self):
        """
        Returns the resolution of the camera

        Returns:
            resolution - A CameraResolution object, the resolution of the
            camera
        """
        with self.resolution_lock:
            return self.resolution

    def _setFov(self, fov):
        """
        INTERNAL METHOD, sets the field of view of the camera

        Parameters:
            fov - The value of the field of view
        """
        try:
            assert type(fov) is int or type(fov) is float
            self.fov = fov

        except AssertionError as e:
            print("Cannot set the camera FOV: " + str(e))

    def _setResolution(self, resolution):
        """
        INTERNAL METHOD, sets the resolution for the camera. The fov of the
        camera has to be setted beforehand

        Parameters:
            resolution - The resolution of the camera (CameraResolution type)
        """
        try:
            with self.resolution_lock:
                assert isinstance(resolution, CameraResolution)
                assert self.fov is not None

                self.resolution = resolution
                self.projection_matrix = pybullet.computeProjectionMatrixFOV(
                    self.fov,
                    self.resolution.width / self.resolution.height,
                    self.near_plane,
                    self.far_plane,
                    physicsClientId=self.physics_client)

        except AssertionError as e:
            print("Cannot set camera resolution: " + str(e))

    def _getCameraImage(self):
        """
        INTERNAL METHOD, Computes the OpenGL virtual camera image. The
        resolution and the projection matrix have to be computed before calling
        this method, or it will crash

        Returns:
            camera_image - The camera image of the OpenGL virtual camera
        """
        _, _, _, _, pos_world, q_world = pybullet.getLinkState(
            self.robot_model,
            self.link.getParentIndex(),
            computeForwardKinematics=True,
            physicsClientId=self.physics_client)

        rotation = pybullet.getMatrixFromQuaternion(q_world)
        forward_vector = [rotation[0], rotation[3], rotation[6]]
        up_vector = [rotation[2], rotation[5], rotation[8]]

        camera_target = [
            pos_world[0] + forward_vector[0] * 10,
            pos_world[1] + forward_vector[1] * 10,
            pos_world[2] + forward_vector[2] * 10]

        view_matrix = pybullet.computeViewMatrix(
            pos_world,
            camera_target,
            up_vector,
            physicsClientId=self.physics_client)

        with self.resolution_lock:
            camera_image = pybullet.getCameraImage(
                self.resolution.width,
                self.resolution.height,
                view_matrix,
                self.projection_matrix,
                renderer=pybullet.ER_BULLET_HARDWARE_OPENGL,
                flags=pybullet.ER_NO_SEGMENTATION_MASK,
                physicsClientId=self.physics_client)

        return camera_image

    def _resetActiveCamera(self):
        """
        INTERNAL METHOD, called when unsubscribing from the active camera, when
        Python is exitted or when the SimulationManager resets/stops a
        simulation instance
        """
        Camera.ACTIVE_CAMERA_ID[self.physics_client] = -1

        if self.extraction_thread.isAlive():
            self.extraction_thread.join()

    def _waitForCorrectImageFormat(self):
        """
        INTERNAL METHOD, to be called after the launch of the exctraction
        thread. Blocking method, that will return when the array retrieved from
        the @getCameraImage is not None and corresponds to the current
        resolution
        """
        try:
            assert self.extraction_thread.isAlive()

            while self.getFrame() is None:
                continue

            while True:
                image = self.getFrame()
                if image.shape[1] == self.resolution.width and\
                   image.shape[0] == self.resolution.height:
                    break

        except AssertionError:
            return


class CameraRgb(Camera):
    """
    Class representing a virtual rgb camera
    """

    def __init__(
            self,
            robot_model,
            link,
            physicsClientId=0,
            resolution=Camera.K_QVGA):
        """
        Constructor

        Parameters:
            robot_model - the pybullet model of the robot
            link - The link (Link type) onto which the camera's attached
            physicsClientId - The id of the simulated instance in which the
            camera is to be spawned
            resolution - The resolution of the camera
        """
        Camera.__init__(
            self,
            robot_model,
            link,
            physicsClientId=physicsClientId)

        self._setFov(67.4)
        self.rgb_image = None

    def subscribe(self, resolution=Camera.K_QVGA):
        """
        Overload @subscribe from @Camera

        Parameters:
            resolution - CameraResolution object, the resolution of the camera.
            By default, the resolution is QVGA
        """
        Camera.subscribe(self, id(self), resolution)
        self.extraction_thread =\
            threading.Thread(target=self._frameExtractionLoop)
        self.extraction_thread.start()
        self._waitForCorrectImageFormat()

    def unsubscribe(self):
        """
        Overload @unsubscribe from @Camera
        """
        Camera.unsubscribe(self, id(self))

    def _frameExtractionLoop(self):
        """
        Frame extraction loop, has to be threaded. The resolution and the FOV
        have to be specified beforehand
        """
        rgb_image = np.zeros((
            self.resolution.height,
            self.resolution.width,
            3))

        try:
            while True:
                assert id(self) == Camera.ACTIVE_CAMERA_ID[self.physics_client]
                camera_image = self._getCameraImage()

                camera_image = np.reshape(
                    camera_image[2],
                    (camera_image[1], camera_image[0], 4))

                rgb_image[:, :, 0] =\
                    (1 - camera_image[:, :, 3]) * camera_image[:, :, 2] +\
                    camera_image[:, :, 3] * camera_image[:, :, 2]

                rgb_image[:, :, 1] =\
                    (1 - camera_image[:, :, 3]) * camera_image[:, :, 1] +\
                    camera_image[:, :, 3] * camera_image[:, :, 1]

                rgb_image[:, :, 2] =\
                    (1 - camera_image[:, :, 3]) * camera_image[:, :, 0] +\
                    camera_image[:, :, 3] * camera_image[:, :, 0]

                rgb_image = rgb_image.astype(np.uint8)

                with self.frame_lock:
                    self.frame = rgb_image.copy()

        except AssertionError:
            return


class CameraDepth(Camera):
    """
    Class representing a virtual depth camera
    """

    def __init__(
            self,
            robot_model,
            link,
            physicsClientId=0,
            resolution=Camera.K_QVGA):
        """
        Constructor

        Parameters:
            robot_model - the pybullet model of the robot
            link - The link (Link type) onto which the camera's attached
            physicsClientId - The id of the simulated instance in which the
            camera is to be spawned
            resolution - The resolution of the camera
        """
        Camera.__init__(
            self,
            robot_model,
            link,
            physicsClientId=physicsClientId)

        self._setFov(70)

    def subscribe(self, resolution=Camera.K_QVGA):
        """
        Overload @subscribe from @Camera

        Parameters:
            resolution - CameraResolution object, the resolution of the camera.
            By default, the resolution is QVGA
        """
        Camera.subscribe(self, id(self), resolution)
        self.extraction_thread =\
            threading.Thread(target=self._frameExtractionLoop)
        self.extraction_thread.start()
        self._waitForCorrectImageFormat()

    def unsubscribe(self):
        """
        Overload @unsubscribe from @Camera
        """
        Camera.unsubscribe(self, id(self))

    def _frameExtractionLoop(self):
        """
        Frame extraction loop, has to be threaded. The resolution and the FOV
        have to be specified beforehand
        """
        far = self.far_plane / 10
        near = self.near_plane / 10
        try:
            while True:
                assert id(self) == Camera.ACTIVE_CAMERA_ID[self.physics_client]
                camera_image = self._getCameraImage()
                depth_image = camera_image[3]

                depth_image = (far * near) /\
                    (far - (far - near) * depth_image)

                image_histogram, bins = np.histogram(
                    depth_image.flatten(),
                    256,
                    normed=True)

                # Cumulative distribution function
                cdf = image_histogram.cumsum()
                # Normalize for 16 bits
                cdf = 65535 * cdf / cdf[-1]

                # Use linear interpolation of cdf to find new pixel values
                image_equalized = np.interp(
                    depth_image.flatten(),
                    bins[:-1],
                    cdf)
                depth_image = image_equalized.reshape(depth_image.shape)

                with self.frame_lock:
                    self.frame = depth_image.astype(np.uint16)

        except AssertionError:
            return
