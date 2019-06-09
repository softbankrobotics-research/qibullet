#!/usr/bin/env python
# coding: utf-8

import atexit
import weakref
import threading


class Controller:
    """
    Abstract class describing a virtual controller
    """
    _instances = set()

    def __init__(self):
        """
        Constructor
        """
        self.control_process = threading.Thread(target=None)
        self._instances.add(weakref.ref(self))
        self._controller_termination = False
        atexit.register(self._terminateController)

    @classmethod
    def _getInstances(cls):
        """
        INTERNAL CLASSMETHOD, get all of the Controller (and daughters)
        instances
        """
        dead = set()

        for ref in cls._instances:
            obj = ref()

            if obj is not None:
                yield obj
            else:
                dead.add(ref)

        cls._instances -= dead

    def _terminateController(self):
        """
        INTERNAL METHOD, can be called to terminate an asynchronous controller.
        Should only be used when killing the simulation
        """
        print(self)
        self._controller_termination = True

        if self.control_process.isAlive():
            self.control_process.join()
