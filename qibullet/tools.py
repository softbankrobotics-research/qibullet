#!/usr/bin/env python
# coding: utf-8
import math
import pybullet

def getDistance(point_a, point_b):
    [x1, y1, z1] = point_a
    [x2, y2, z2] = point_b
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)

def getOrientation(quaternion_a, quaternion_b):
    theta_a = pybullet.getEulerFromQuaternion(quaternion_a)
    theta_b = pybullet.getEulerFromQuaternion(quaternion_b)
    return abs(theta_b[-1] - theta_a[-1])