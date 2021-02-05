#!/usr/bin/env python
# coding: utf-8

import math
import time
import pybullet
import threading
import numpy as np
from qibullet.link import Link
from qibullet.sensor import Sensor


class CameraResolution:
    """
    Structure for the camera resolutions
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
        try:
            assert self.width == resolution.width
            assert self.height == resolution.height
            return True

        except AssertionError:
            return False


class Camera(Sensor):
    """
    Class representing a virtual camera
    """
    CAMERA_HANDLES = dict()
    HANDLES_LOCK = threading.Lock()

    K_QQVGA = CameraResolution(160, 120)
    K_QVGA = CameraResolution(320, 240)
    K_VGA = CameraResolution(640, 480)
    K_QQ720p = CameraResolution(320, 180)
    K_Q720p = CameraResolution(640, 360)
    K_720p = CameraResolution(1280, 720)

    def __init__(
            self,
            robot_model,
            camera_id,
            camera_link,
            hfov,
            vfov,
            physicsClientId=0,
            near_plane=0.01,
            far_plane=100):
        """
        Constructor

        Parameters:
            robot_model - The pybullet model of the robot
            camera_id - an int describing the id of the camera
            camera_link - The link (Link type) onto which the camera's attached
            hfov - The value of the horizontal field of view angle (in degrees)
            vfov - The value of the vertical field of view angle (in degrees)
            physicsClientId - The id of the simulated instance in which the
            camera is to be spawned
            near_plane - The near plane distance
            far_plane - The far plane distance
        """
        Sensor.__init__(self, robot_model, physicsClientId)
        self.camera_id = camera_id
        self.camera_link = camera_link
        self.near_plane = near_plane
        self.far_plane = far_plane
        self.frame = None
        self.projection_matrix = None
        self.resolution = None
        self.hfov = None
        self.vfov = None
        self.intrinsic_matrix = None
        self.resolution_lock = threading.Lock()
        self._setFov(hfov, vfov)

    def subscribe(self, resolution, fps=30.0):
        """
        Subscribing method for the camera (the FOV has to be specified
        beforehand). This method will launch the frame extraction loop process
        of the camera in a separate thread. This method will return a camera
        handle, needed to retrieve camera frames, get the camera resolution,
        etc. and unsubscribe from it

        Returns:
            handle - The handle of the camera
            fps - The number of frames per second requested for the video
            source (strictly positive float of int)
        """
        # Lets the module process thread die before launching another one, if
        # the same camera calls the subscribing method for a second time
        self._terminateModule()
        self._module_termination = False

        self._setResolution(resolution)
        self._setFps(fps)

        self.module_process =\
            threading.Thread(target=self._frameExtractionLoop)
        self.module_process.start()
        self._waitForCorrectImageFormat()

        # Add the camera and it's handle to the handles dict
        Camera._addCameraHandle(id(self), self)
        return id(self)

    def unsubscribe(self):
        """
        Method stopping the frame retreival thread for a camera.

        Returns:
            success - Boolean, True if unsubscribed successfully, False
            otherwise
        """
        try:
            self._removeCameraHandle(id(self))
            self._terminateModule()
            self.frame = None
            return True

        except KeyError:
            return False

    def getCameraId(self):
        """
        Returns the id of the camera. WARNING, the id of the camera is not the
        id of the Python object corresponding to the camera, the id value is
        defined by the user and specific to each camera of each robot (
        PepperVirtual.ID_CAMERA_TOP for instance).

        Returns:
            camera_id - The id of the camera
        """
        return self.camera_id

    def getCameraLink(self):
        """
        Returns the link to which the camera is attached, as a qiBullet Link
        object

        Returns:
            camera_link - The link to which the camera is attached
        """
        return self.camera_link

    def getFrame(self):
        """
        Returns the current frame

        Returns:
            frame_copy - a copy of the current frame
        """
        try:
            assert self.frame is not None
            return self.frame.copy()

        except AssertionError:
            return None

    def isActive(self):
        """
        Specifies if the camera is active or not (if a handle exists for the
        current camera)

        Returns:
            is_active - Boolean, True if the camera is subscribed to, False
            otherwise
        """
        if id(self) in Camera._getCameraHandlesDict():
            return True
        else:
            return False

    def getResolution(self):
        """
        Returns the resolution of the camera

        Returns:
            resolution - A CameraResolution object, the resolution of the
            camera
        """
        with self.resolution_lock:
            return self.resolution

    def getFps(self):
        """
        Returns the framerate of the camera in number of frames per second.
        This method simply wraps the @getFrequency method of Sensor, and will
        return the frequency of the camera in Hz

        Returns:
            fps - The number of frames per second for the camera (frequency of
            the camera in Hz)
        """
        return self.getFrequency()

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
                assert self.hfov is not None and self.vfov is not None

                self.resolution = resolution
                self.projection_matrix = pybullet.computeProjectionMatrix(
                    left=-math.tan(math.pi*self.hfov/360.0)*self.near_plane,
                    right=math.tan(math.pi*self.hfov/360.0)*self.near_plane,
                    bottom=-math.tan(math.pi*self.vfov/360.0)*self.near_plane,
                    top=math.tan(math.pi*self.vfov/360.0)*self.near_plane,
                    nearVal=self.near_plane,
                    farVal=self.far_plane,
                    physicsClientId=self.getPhysicsClientId())

                self.intrinsic_matrix = [
                    self.projection_matrix[0]*self.resolution.width/2.0,
                    0.0,
                    (1-self.projection_matrix[8])*self.resolution.width/2.0,
                    0.0,
                    self.projection_matrix[5]*self.resolution.height/2.0,
                    (1-self.projection_matrix[9])*self.resolution.height/2.0,
                    0.0,
                    0.0,
                    1.0]

        except AssertionError as e:
            raise pybullet.error("Cannot set camera resolution: " + str(e))

    def _setFps(self, fps):
        """
        INTERNAL METHOD, sets the framerate of the camera. This method actually
        specifies the frequency of the camera, and simply wraps the
        @setFrequency method defined in the Sensor class (for API consistency)

        Parameters:
            fps - The framerate of the camera (ultimately its frequency in Hz,
            formated as a strictly positive float or int)
        """
        self.setFrequency(fps)

    def _setFov(self, hfov, vfov):
        """
        INTERNAL METHOD, sets the field of view of the camera

        Parameters:
            hfov - The value of the horizontal field of view angle (in degrees)
            vfov - The value of the vertical field of view angle (in degrees)
        """
        try:
            assert isinstance(hfov, float) or isinstance(hfov, int)
            assert isinstance(vfov, float) or isinstance(vfov, int)
            self.hfov = hfov
            self.vfov = vfov

        except AssertionError as e:
            print("Cannot set the camera FOV: " + str(e))

    def _getCameraIntrinsics(self):
        """
        INTERNAL METHOD, Returns the intrinsic camera matrix, formatted as:
        K = [fx, 0, cx, 0, fy, cy, 0.0, 0.0, 1.0]. For more details, see
        http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/CameraInfo.html

        Returns:
            intrinsic_matrix - The intrinsic camera matrix formatted as a list,
            or None if the intrinsic camera matrix is None
        """
        with self.resolution_lock:
            try:
                assert self.intrinsic_matrix is not None
                return list(self.intrinsic_matrix)

            except AssertionError:
                print("Cannot get intrinsics, resolution not defined")
                return None

    def _getCameraImage(self):
        """
        INTERNAL METHOD, Computes the OpenGL virtual camera image. The
        resolution and the projection matrix have to be computed before calling
        this method, or it will crash

        Returns:
            camera_image - The camera image of the OpenGL virtual camera
        """
        _, _, _, _, pos_world, q_world = pybullet.getLinkState(
            self.getRobotModel(),
            self.camera_link.getParentIndex(),
            computeForwardKinematics=False,
            physicsClientId=self.getPhysicsClientId())

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
            physicsClientId=self.getPhysicsClientId())

        with self.resolution_lock:
            camera_image = pybullet.getCameraImage(
                self.resolution.width,
                self.resolution.height,
                view_matrix,
                self.projection_matrix,
                renderer=pybullet.ER_BULLET_HARDWARE_OPENGL,
                flags=pybullet.ER_NO_SEGMENTATION_MASK,
                physicsClientId=self.getPhysicsClientId())

        return camera_image

    def _waitForCorrectImageFormat(self):
        """
        INTERNAL METHOD, to be called after the launch of the exctraction
        thread. Blocking method, that will return when the array retrieved from
        the @getCameraImage is not None and corresponds to the current
        resolution
        """
        try:
            assert self.isAlive()

            while self.getFrame() is None:
                continue

            while True:
                image = self.getFrame()
                if image.shape[1] == self.resolution.width and\
                   image.shape[0] == self.resolution.height:
                    break

        except AssertionError:
            return

    def _frameExtractionLoop(self):
        """
        ABSTRACT METHOD, To be specified in each dauhter class, frame
        extraction loop method. This method is threaded. The resolution and the
        FOV have to be specified beforehand
        """
        raise NotImplementedError

    @classmethod
    def _addCameraHandle(cls, handle, camera):
        """
        INTERNAL METHOD, Adds a camera and it's handle to the CAMERA_HANDLES
        dict

        Parameters:
            handle - The camera handle
            camera - The corresponding camera
        """
        with cls.HANDLES_LOCK:
            cls.CAMERA_HANDLES[handle] = camera

    @classmethod
    def _removeCameraHandle(cls, handle):
        """
        INTERNAL METHOD, Remove a camera and it's handle from the
        CAMERA_HANDLES dict. Throws a KeyError if the handle doesn't exist

        Parameters:
            handle - The camera handle
        """
        with cls.HANDLES_LOCK:
            del cls.CAMERA_HANDLES[handle]

    @classmethod
    def _getCameraFromHandle(cls, handle):
        """
        INTERNAL METHOD, Returns the camera associated to the specified handle.
        Throws a KeyError if the handle doesn't exist

        Parameters:
            handle - The camera handle

        Returns:
            camera - The camera object associated to the handle
        """
        with cls.HANDLES_LOCK:
            return cls.CAMERA_HANDLES[handle]

    @classmethod
    def _getCameraHandlesDict(cls):
        """
        INTERNAL METHOD, Returns a COPY of the CAMERA_HANDLES dict

        Returns:
            camera_handles - A copy of the CAMERA_HANDLES dict
        """
        with cls.HANDLES_LOCK:
            return cls.CAMERA_HANDLES.copy()


class CameraRgb(Camera):
    """
    Class representing a virtual rgb camera
    """

    def __init__(
            self,
            robot_model,
            camera_id,
            camera_link,
            hfov,
            vfov,
            physicsClientId=0,
            resolution=Camera.K_QVGA):
        """
        Constructor

        Parameters:
            robot_model - the pybullet model of the robot
            camera_id - an int describing the id of the camera
            camera_link - The link (Link type) onto which the camera's attached
            hfov - The value of the horizontal field of view angle (in degrees)
            vfov - The value of the vertical field of view angle (in degrees)
            physicsClientId - The id of the simulated instance in which the
            camera is to be spawned
            resolution - The resolution of the camera
        """
        Camera.__init__(
            self,
            robot_model,
            camera_id,
            camera_link,
            hfov,
            vfov,
            physicsClientId=physicsClientId)

    def _frameExtractionLoop(self):
        """
        Frame extraction loop, has to be threaded. The resolution and the FOV
        have to be specified beforehand
        """
        period = 1.0 / self.getFps()
        sampling_time = time.time()

        rgb_image = np.zeros((
            self.resolution.height,
            self.resolution.width,
            3))

        while not self._module_termination:
            current_time = time.time()

            if current_time - sampling_time < period:
                continue

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

            self.frame = rgb_image.astype(np.uint8)
            sampling_time = current_time


class CameraDepth(Camera):
    """
    Class representing a virtual depth camera
    """

    def __init__(
            self,
            robot_model,
            camera_id,
            camera_link,
            hfov,
            vfov,
            physicsClientId=0,
            resolution=Camera.K_QVGA):
        """
        Constructor

        Parameters:
            robot_model - the pybullet model of the robot
            camera_id - an int describing the id of the camera
            camera_link - The link (Link type) onto which the camera's attached
            hfov - The value of the horizontal field of view angle (in degrees)
            vfov - The value of the vertical field of view angle (in degrees)
            physicsClientId - The id of the simulated instance in which the
            camera is to be spawned
            resolution - The resolution of the camera
        """
        Camera.__init__(
            self,
            robot_model,
            camera_id,
            camera_link,
            hfov,
            vfov,
            near_plane=0.4,
            far_plane=8,
            physicsClientId=physicsClientId)

    def _frameExtractionLoop(self):
        """
        Frame extraction loop, has to be threaded. The resolution and the FOV
        have to be specified beforehand
        """
        period = 1.0 / self.getFps()
        sampling_time = time.time()

        while not self._module_termination:
            current_time = time.time()

            if current_time - sampling_time < period:
                continue

            camera_image = self._getCameraImage()
            depth_image = np.reshape(
                camera_image[3],
                (camera_image[1], camera_image[0]))

            depth_image = (self.far_plane * self.near_plane) /\
                (self.far_plane - (self.far_plane - self.near_plane) *
                    depth_image)

            depth_image *= 1000
            self.frame = depth_image.astype(np.uint16)
            sampling_time = current_time
