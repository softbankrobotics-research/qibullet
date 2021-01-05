#!/usr/bin/env python
# coding: utf-8
import unittest
import pybullet
from qibullet.imu import Imu
from qibullet import SimulationManager
from qibullet import PepperVirtual
from qibullet import NaoVirtual
from qibullet import RomeoVirtual
from qibullet.robot_virtual import RobotVirtual


class ImuTest(unittest.TestCase):
    """
    Unittests for the Inertial Unit (Imu object)
    """

    def test_subscribe_unsubscribe(self):
        """
        Test the @subscribe and @unsubscribe methods of the Imu class.
        """
        self.assertFalse(ImuTest.robot.imu.isAlive())
        ImuTest.robot.imu.subscribe()
        self.assertTrue(ImuTest.robot.imu.isAlive())
        ImuTest.robot.imu.subscribe()
        self.assertTrue(ImuTest.robot.imu.isAlive())
        ImuTest.robot.imu.unsubscribe()
        self.assertFalse(ImuTest.robot.imu.isAlive())
        ImuTest.robot.imu.unsubscribe()
        self.assertFalse(ImuTest.robot.imu.isAlive())

    def test_get_gyroscope_values(self):
        """
        Test the @getGyroscopeValues method of the Imu class
        """
        ImuTest.robot.imu.subscribe()
        values = ImuTest.robot.imu.getGyroscopeValues()
        ImuTest.robot.imu.unsubscribe()

        self.assertEqual(len(values), 3)
        all(self.assertIsInstance(value, float) for value in values)

    def test_get_accelerometer_values(self):
        """
        Test the @getAccelerometerValues method of the Imu class
        """
        ImuTest.robot.imu.subscribe()
        values = ImuTest.robot.imu.getAccelerometerValues()
        ImuTest.robot.imu.unsubscribe()

        self.assertEqual(len(values), 3)
        all(self.assertIsInstance(value, float) for value in values)

    def test_get_values(self):
        """
        Test the @getValues method of the Imu class
        """
        ImuTest.robot.imu.subscribe()
        gyro_values, acc_values = ImuTest.robot.imu.getValues()
        ImuTest.robot.imu.unsubscribe()

        self.assertEqual(len(gyro_values), 3)
        self.assertEqual(len(acc_values), 3)
        all(self.assertIsInstance(value, float) for value in gyro_values)
        all(self.assertIsInstance(value, float) for value in acc_values)

    def test_subscribe_unsubscribe_imu(self):
        """
        Test the @subscribeImu and @unsubscribeImu methods of the RobotVirtual
        class
        """
        self.assertFalse(ImuTest.robot.imu.isAlive())

        with self.assertRaises(pybullet.error):
            ImuTest.robot.subscribeImu(frequency=-3)

        self.assertFalse(ImuTest.robot.imu.isAlive())
        ImuTest.robot.subscribeImu(frequency=None)
        ImuTest.robot.subscribeImu()
        ImuTest.robot.subscribeImu(frequency=100)
        self.assertTrue(ImuTest.robot.imu.isAlive())
        ImuTest.robot.unsubscribeImu()
        self.assertFalse(ImuTest.robot.imu.isAlive())
        ImuTest.robot.unsubscribeImu()
        self.assertFalse(ImuTest.robot.imu.isAlive())

    def test_get_imu_gyroscope_values(self):
        """
        Test the @getImuGyroscopeValues method of the RobotVirtual class
        """
        ImuTest.robot.imu.subscribe()
        values = ImuTest.robot.getImuGyroscopeValues()
        ImuTest.robot.imu.unsubscribe()

        self.assertEqual(len(values), 3)
        all(self.assertIsInstance(value, float) for value in values)

    def test_get_imu_accelerometer_values(self):
        """
        Test the @getImuAccelerometerValues method of the RobotVirtual class
        """
        ImuTest.robot.imu.subscribe()
        values = ImuTest.robot.getImuAccelerometerValues()
        ImuTest.robot.imu.unsubscribe()

        self.assertEqual(len(values), 3)
        all(self.assertIsInstance(value, float) for value in values)

    def test_get_imu_values(self):
        """
        Test the @getImuValues method of the RobotVirtual class
        """
        ImuTest.robot.imu.subscribe()
        gyro_values, acc_values = ImuTest.robot.getImuValues()
        ImuTest.robot.imu.unsubscribe()

        self.assertEqual(len(gyro_values), 3)
        self.assertEqual(len(acc_values), 3)
        all(self.assertIsInstance(value, float) for value in gyro_values)
        all(self.assertIsInstance(value, float) for value in acc_values)

    def test_get_imu(self):
        """
        Test the @getImu method of the RobotVirtual class
        """
        imu = ImuTest.robot.getImu()
        self.assertIsInstance(imu, Imu)


class PepperImuTest(ImuTest):
    """
    Unittests for the control of Pepper virtual's joints
    """

    @classmethod
    def setUpClass(cls):
        """
        Launches a simulation and spawns the Pepper virtual robot
        """
        ImuTest.simulation = SimulationManager()
        ImuTest.client = ImuTest.simulation.launchSimulation(
            gui=False)

        ImuTest.robot = ImuTest.simulation.spawnPepper(
            ImuTest.client,
            spawn_ground_plane=True)

    @classmethod
    def tearDownClass(cls):
        """
        Stops the simulation
        """
        ImuTest.simulation.stopSimulation(
            ImuTest.client)

    def test_subscribe_unsubscribe(self):
        ImuTest.test_subscribe_unsubscribe(self)

    def test_get_gyroscope_values(self):
        ImuTest.test_get_gyroscope_values(self)

    def test_get_accelerometer_values(self):
        ImuTest.test_get_accelerometer_values(self)

    def test_get_values(self):
        ImuTest.test_get_values(self)

    def test_subscribe_unsubscribe_imu(self):
        ImuTest.test_subscribe_unsubscribe_imu(self)

    def test_get_imu_gyroscope_values(self):
        ImuTest.test_get_imu_gyroscope_values(self)

    def test_get_imu_accelerometer_values(self):
        ImuTest.test_get_imu_accelerometer_values(self)

    def test_get_imu_values(self):
        ImuTest.test_get_imu_values(self)

    def test_get_imu(self):
        ImuTest.test_get_imu(self)


class NaoImuTest(ImuTest):
    """
    Unittests for the control of Nao virtual's joints
    """

    @classmethod
    def setUpClass(cls):
        """
        Launches a simulation and spawns the NAO virtual robot
        """
        ImuTest.simulation = SimulationManager()
        ImuTest.client = ImuTest.simulation.launchSimulation(
            gui=False)

        ImuTest.robot = ImuTest.simulation.spawnNao(
            ImuTest.client,
            spawn_ground_plane=True)

    @classmethod
    def tearDownClass(cls):
        """
        Stops the simulation
        """
        ImuTest.simulation.stopSimulation(
            ImuTest.client)

    def test_subscribe_unsubscribe(self):
        ImuTest.test_subscribe_unsubscribe(self)

    def test_get_gyroscope_values(self):
        ImuTest.test_get_gyroscope_values(self)

    def test_get_accelerometer_values(self):
        ImuTest.test_get_accelerometer_values(self)

    def test_get_values(self):
        ImuTest.test_get_values(self)

    def test_subscribe_unsubscribe_imu(self):
        ImuTest.test_subscribe_unsubscribe_imu(self)

    def test_get_imu_gyroscope_values(self):
        ImuTest.test_get_imu_gyroscope_values(self)

    def test_get_imu_accelerometer_values(self):
        ImuTest.test_get_imu_accelerometer_values(self)

    def test_get_imu_values(self):
        ImuTest.test_get_imu_values(self)

    def test_get_imu(self):
        ImuTest.test_get_imu(self)


class RomeoImuTest(ImuTest):
    """
    Unittests for the control of Nao virtual's joints
    """

    @classmethod
    def setUpClass(cls):
        """
        Launches a simulation and spawns the NAO virtual robot
        """
        ImuTest.simulation = SimulationManager()
        ImuTest.client = ImuTest.simulation.launchSimulation(
            gui=False)

        ImuTest.robot = ImuTest.simulation.spawnRomeo(
            ImuTest.client,
            spawn_ground_plane=True)

    @classmethod
    def tearDownClass(cls):
        """
        Stops the simulation
        """
        ImuTest.simulation.stopSimulation(
            ImuTest.client)

    def test_subscribe_unsubscribe(self):
        ImuTest.test_subscribe_unsubscribe(self)

    def test_get_gyroscope_values(self):
        ImuTest.test_get_gyroscope_values(self)

    def test_get_accelerometer_values(self):
        ImuTest.test_get_accelerometer_values(self)

    def test_get_values(self):
        ImuTest.test_get_values(self)

    def test_subscribe_unsubscribe_imu(self):
        ImuTest.test_subscribe_unsubscribe_imu(self)

    def test_get_imu_gyroscope_values(self):
        ImuTest.test_get_imu_gyroscope_values(self)

    def test_get_imu_accelerometer_values(self):
        ImuTest.test_get_imu_accelerometer_values(self)

    def test_get_imu_values(self):
        ImuTest.test_get_imu_values(self)

    def test_get_imu(self):
        ImuTest.test_get_imu(self)


class DummyImuTest(unittest.TestCase):
    """
    Unittests for the behaviour of the Imu related methods of RobotVirtual,
    when the Imu is None (default value)
    """

    @classmethod
    def setUpClass(cls):
        """
        Launches a simulation and spawns the NAO virtual robot
        """
        DummyImuTest.simulation = SimulationManager()
        DummyImuTest.client = DummyImuTest.simulation.launchSimulation(
            gui=False)

        DummyImuTest.robot = DummyRobot()

    @classmethod
    def tearDownClass(cls):
        """
        Stops the simulation
        """
        DummyImuTest.simulation.stopSimulation(
            DummyImuTest.client)

    def test_imu_none(self):
        """
        Test the behaviour of the Imu related methods of RobotVirtual when the
        Imu of the robot doesn't exist (set to None)
        """
        with self.assertRaises(pybullet.error):
            DummyImuTest.robot.subscribeImu()

        with self.assertRaises(pybullet.error):
            DummyImuTest.robot.unsubscribeImu()

        with self.assertRaises(pybullet.error):
            DummyImuTest.robot.getImuGyroscopeValues()

        with self.assertRaises(pybullet.error):
            DummyImuTest.robot.getImuAccelerometerValues()

        with self.assertRaises(pybullet.error):
            DummyImuTest.robot.getImuValues()

        self.assertIsNone(DummyImuTest.robot.getImu())

    def test_imu_creation(self):
        """
        Test the behaviour of the Imu constructor when a wrongful frequency is
        specified
        """
        imu = Imu(-1, None, 10)
        self.assertIsInstance(imu.getFrequency(), float)
        self.assertEqual(imu.getFrequency(), 10.0)

        with self.assertRaises(pybullet.error):
            imu = Imu(-1, None, None)

        with self.assertRaises(pybullet.error):
            imu = Imu(-1, None, -3)


class DummyRobot(RobotVirtual):
    """
    Dummy robot class
    """
    def __init__(self):
        """
        Constructor
        """
        RobotVirtual.__init__(self, "wrong_description_filepath")


if __name__ == "__main__":
    unittest.main()
