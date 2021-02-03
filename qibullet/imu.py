#!/usr/bin/env python
# coding: utf-8

import time
import pybullet
import threading
from qibullet.sensor import Sensor


class Imu(Sensor):
    """
    Class representing a virtual inertial unit
    """

    def __init__(self, robot_model, imu_link, frequency, physicsClientId=0):
        """
        Constructor. If the specified frequency is not a strictly positive int
        or float, the constructor will raise a pybullet error

        Parameters:
            robot_model - The pybullet model of the robot
            imu_link - The Link object corresponding to the link the IMU is
            attached to
            frequency - The frequency of the IMU, in Hz
            physicsClientId - The id of the simulated instance in which the
            IMU should be spawned
        """
        Sensor.__init__(
            self,
            robot_model,
            physicsClientId)

        self.setFrequency(frequency)

        self.imu_link = imu_link
        self.angular_velocity = [0.0, 0.0, 0.0]
        self._linear_velocity = [0.0, 0.0, 0.0]
        self.linear_acceleration = [0.0, 0.0, 0.0]
        self.values_lock = threading.Lock()

    def subscribe(self):
        """
        Subscribe to the IMU, activating the IMU scan process
        """
        if self.isAlive():
            return

        self._module_termination = False
        self.module_process = threading.Thread(target=self._imuScan)
        self.module_process.start()

    def unsubscribe(self):
        """
        Unsubscribe from the IMU, deactivating the IMU scan process
        """
        if self.isAlive():
            self._terminateModule()

    def getGyroscopeValues(self):
        """
        Returns the angular velocity of the IMU in rad/s in the
        world frame

        Returns:
            angular_velocity - The angular velocity in rad/s
        """
        with self.values_lock:
            return self.angular_velocity

    def getAccelerometerValues(self):
        """
        Returns the linear acceleration of the IMU in m/s^2 in the
        world frame

        Returns:
            linear_acceleration - The linear acceleration in m/s^2
        """
        with self.values_lock:
            return self.linear_acceleration

    def getValues(self):
        """
        Returns the values of the gyroscope and the accelerometer of the IMU
        (angular_velocity, linear_acceleration) in the world frame

        Returns:
            angular_velocity - The angular velocity values in rad/s
            linear_acceleration - The linear acceleration values in m/s^2
        """
        with self.values_lock:
            return self.angular_velocity, self.linear_acceleration

    def _imuScan(self):
        """
        INTERNAL METHOD, retrieves and update the IMU data
        """
        period = 1.0 / self.getFrequency()
        sampling_time = time.time()

        while not self._module_termination:
            current_time = time.time()

            if current_time - sampling_time < period:
                continue

            link_state = pybullet.getLinkState(
                self.getRobotModel(),
                self.imu_link.getIndex(),
                computeLinkVelocity=True,
                physicsClientId=self.getPhysicsClientId())

            with self.values_lock:
                self.angular_velocity = link_state[7]
                self.linear_acceleration = [
                    (i - j) / (time.time() - sampling_time) for i, j in zip(
                        link_state[6],
                        self._linear_velocity)]

                self._linear_velocity = link_state[6]
                sampling_time = current_time
