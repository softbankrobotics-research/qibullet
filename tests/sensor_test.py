#!/usr/bin/env python
# coding: utf-8
import unittest
import pybullet
from qibullet import SimulationManager
from qibullet.sensor import Sensor


class SensorTest(unittest.TestCase):
    """
    Unittests for the Sensor class functionalities (will use a dummy sensor
    and a dummy robot for the test, the dummy sensor will inherit from Sensor)
    """

    @classmethod
    def setUpClass(cls):
        """
        Launches a simulation and spawns a Pepper virtual robot
        """
        SensorTest.simulation = SimulationManager()
        SensorTest.client = SensorTest.simulation.launchSimulation(
            gui=False)

        SensorTest.sensor = DummySensor(SensorTest.client)

    @classmethod
    def tearDownClass(cls):
        """
        Stops the simulation
        """
        SensorTest.simulation.stopSimulation(
            SensorTest.client)

    def test_set_get_frequency(self):
        """
        Test the @setFrequency and @getFrequency method of Sensor
        """
        self.assertIsNone(SensorTest.sensor.getFrequency())

        with self.assertRaises(pybullet.error):
            SensorTest.sensor.setFrequency(None)

        self.assertIsNone(SensorTest.sensor.getFrequency())
        SensorTest.sensor.setFrequency(5)
        self.assertIsInstance(SensorTest.sensor.getFrequency(), float)
        self.assertEqual(SensorTest.sensor.getFrequency(), 5.0)
        SensorTest.sensor.setFrequency(10.0)
        self.assertIsInstance(SensorTest.sensor.getFrequency(), float)
        self.assertEqual(SensorTest.sensor.getFrequency(), 10.0)

        with self.assertRaises(pybullet.error):
            SensorTest.sensor.setFrequency(0)

        self.assertEqual(SensorTest.sensor.getFrequency(), 10.0)

        with self.assertRaises(pybullet.error):
            SensorTest.sensor.setFrequency(-3.0)

        self.assertEqual(SensorTest.sensor.getFrequency(), 10.0)

    def test_subscribe(self):
        """
        Test the @subscribe method of Sensor, should raise a
        NotImplementedError
        """
        with self.assertRaises(NotImplementedError):
            SensorTest.sensor.subscribe()

    def test_unsubscribe(self):
        """
        Test the @unsubscribe method of Sensor, should raise a
        NotImplementedError
        """
        with self.assertRaises(NotImplementedError):
            SensorTest.sensor.unsubscribe()


class DummySensor(Sensor):
    """
    Dummy sensor class
    """
    def __init__(self, physics_client):
        """
        Constructor
        """
        Sensor.__init__(self, -1, physics_client)
