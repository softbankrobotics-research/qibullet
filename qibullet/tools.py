#!/usr/bin/env python
# coding: utf-8
import math
import pybullet


def getDistance(point_a, point_b):
    [x1, y1, z1] = point_a
    [x2, y2, z2] = point_b
    return int(math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2) * 100)\
        / 100.0


def getOrientation(theta_a, theta_b):
    return theta_b[-1] - theta_a[-1]


def computeVelocity(acc, vel_min, vel_max, dist_traveled, dist_remained):
    distance_acc = (vel_max * vel_max) / (2 * acc)
    if dist_traveled < distance_acc:
        vel_computed = (vel_max - vel_min) *\
            dist_traveled / distance_acc + vel_min
    if dist_traveled >= distance_acc:
        vel_computed = vel_max
    if dist_remained < distance_acc:
        vel_computed = (vel_max - vel_min) *\
            dist_remained / distance_acc + vel_min
    return vel_computed
