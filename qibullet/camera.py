#!/usr/bin/env python
# coding: utf-8

import math
import time
import atexit
import weakref
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
        if self.width == resolution.width and self.height == resolution.height:
            return True
        else:
            return False


class Camera(Sensor):
    """
    Class representing a virtual camera
    """
    ACTIVE_OBJECT_ID = dict()

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

        if self.physics_client not in Camera.ACTIVE_OBJECT_ID.keys():
            Camera.ACTIVE_OBJECT_ID[self.physics_client] = -1

    def subscribe(self, resolution):
        """
        Subscribing method for the camera. The FOV has to be specified
        beforehand
        """
        # Lets the module process thread die before launching another one if
        # the same camera calls the subscribing method for a second time
        self._terminateModule()
        self._module_termination = False

        self._setResolution(resolution)
        Camera.ACTIVE_OBJECT_ID[self.physics_client] = id(self)

    def unsubscribe(self):
        """
        Method stopping the frame retreival thread for a camera
        """
        if Camera.ACTIVE_OBJECT_ID[self.physics_client] == id(self):
            Camera.ACTIVE_OBJECT_ID[self.physics_client] = -1
            self._terminateModule()
            self.frame = None

    def getCameraId(self):
        """
        Returns the id of the camera. WARNING, the id of the camera is not the
        id of the Python object corresponding to the camera, the id value is
        defined by the user and specific to each camera of each robot.

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
        Specifies if the camera is active or not

        Returns:
            is_active - Boolean, True if the camera is subscribed to, False
            otherwise
        """
        return id(self) == Camera.ACTIVE_OBJECT_ID[self.physics_client]

    def getResolution(self):
        """
        Returns the resolution of the camera

        Returns:
            resolution - A CameraResolution object, the resolution of the
            camera
        """
        with self.resolution_lock:
            return self.resolution

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
                    physicsClientId=self.physics_client)

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
            print("Cannot set camera resolution: " + str(e))

    def _getCameraIntrinsics(self):
        """
        INTERNAL METHOD, Returns the intrinsic camera matrix, formatted as:
        K = [fx, 0, cx, 0, fy, cy, 0.0, 0.0, 1.0]. For more details, see
        http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/CameraInfo.html

        Returns:
            intrinsic_matrix - The intrinsic camera matrix formatted as a list
        """
        with self.resolution_lock:
            try:
                assert self.intrinsic_matrix is not None

            except AssertionError:
                print("Cannot get intrinsics, resolution not defined")
            finally:
                return list(self.intrinsic_matrix)

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
            self.camera_link.getParentIndex(),
            computeForwardKinematics=False,
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
        Camera.ACTIVE_OBJECT_ID[self.physics_client] = -1

        if self.module_process.isAlive():
            self.module_process.join()

    def _waitForCorrectImageFormat(self):
        """
        INTERNAL METHOD, to be called after the launch of the exctraction
        thread. Blocking method, that will return when the array retrieved from
        the @getCameraImage is not None and corresponds to the current
        resolution
        """
        try:
            assert self.module_process.isAlive()

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

    def subscribe(self, resolution=Camera.K_QVGA):
        """
        Overload @subscribe from @Camera

        Parameters:
            resolution - CameraResolution object, the resolution of the camera.
            By default, the resolution is QVGA
        """
        Camera.subscribe(self, resolution)
        self.module_process =\
            threading.Thread(target=self._frameExtractionLoop)
        self.module_process.start()
        self._waitForCorrectImageFormat()

    def unsubscribe(self):
        """
        Overload @unsubscribe from @Camera
        """
        Camera.unsubscribe(self)

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
            while not self._module_termination:
                assert id(self) == Camera.ACTIVE_OBJECT_ID[self.physics_client]
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

        except AssertionError:
            return


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

    def subscribe(self, resolution=Camera.K_QVGA):
        """
        Overload @subscribe from @Camera

        Parameters:
            resolution - CameraResolution object, the resolution of the camera.
            By default, the resolution is QVGA
        """
        Camera.subscribe(self, resolution)
        self.module_process =\
            threading.Thread(target=self._frameExtractionLoop)
        self.module_process.start()
        self._waitForCorrectImageFormat()

    def unsubscribe(self):
        """
        Overload @unsubscribe from @Camera
        """
        Camera.unsubscribe(self)

    def _frameExtractionLoop(self):
        """
        Frame extraction loop, has to be threaded. The resolution and the FOV
        have to be specified beforehand
        """
        try:
            while not self._module_termination:
                assert id(self) == Camera.ACTIVE_OBJECT_ID[self.physics_client]
                camera_image = self._getCameraImage()
                depth_image = camera_image[3]

                depth_image = (self.far_plane * self.near_plane) /\
                    (self.far_plane - (self.far_plane - self.near_plane) *
                        depth_image)

                depth_image *= 1000
                self.frame = depth_image.astype(np.uint16)

        except AssertionError:
            return
