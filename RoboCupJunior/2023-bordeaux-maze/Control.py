# SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
# Required Notice: Copyright (c) 2026 George Zhang — https://github.com/TheYellowDuck

"""Standalone LiDAR-alignment / movement test script.

Helper routines for squaring the robot up against maze walls using the spinning
LiDAR: estimate the heading error from the distance scan (``getAngle``), rotate to
a target heading (``turn`` / ``fixAngle``), read which sides have walls
(``checkWalls``), and drive one cell (``move``). Run directly, it performs a single
forward move as a quick drive test.
"""

from pyb2 import *
import time, math

ld = Lidar(3)
motor = SServo(1)
anglePrecision = 6.5


def getAngle(ds):
    """Estimate heading error (degrees) from a LiDAR distance scan ``ds``.

    Returns the signed offset from the nearest 90-degree alignment, where the sign
    indicates which way the robot is rotated relative to the walls.
    """
    angle = 0.0
    count = 0
    for i in ds:
        a = i % 90
        b = 90 - a
        if a <= b:
            count += 1
        else:
            count -= 1
        angle += min(i % 90, 90 - i % 90)
    angle /= len(ds)
    if count > 0:
        return angle
    return -angle


def turn(angle, d):
    """Rotate in place to a target heading.

    ``d`` selects the turn: 1 = right (toward +90), 2 = about-face (180),
    3 = left (toward -90). Turn duration is derived from the current ``angle`` and
    the empirically-tuned ``anglePrecision`` constant.
    """
    speed = 1000
    t = 0
    if d == 1:
        t = abs(round((90 - angle) * anglePrecision * anglePrecision))
        motor.set_speeds([-speed, -speed, -speed, -speed])
    if d == 2:
        t = abs(round((180 - abs(angle)) * anglePrecision * anglePrecision))
        if angle > anglePrecision:
            motor.set_speeds([speed, speed, speed, speed])
        if angle < anglePrecision:
            motor.set_speeds([-speed, -speed, -speed, -speed])
    if d == 3:
        t = abs(round((-90 - angle) * anglePrecision * anglePrecision))
        motor.set_speeds([speed, speed, speed, speed])
    time.sleep_ms(t)
    motor.set_speeds([0, 0, 0, 0])


def fixPos(angle, ds):
    """Estimate the robot's offset from the cell centre on both axes.

    Projects the distance scan ``ds`` onto the wall normals around ``angle`` and the
    opposite wall (``angle + 180``), averaging over a +/-30-degree window.
    """
    center = round(angle)
    distance1 = 0.0
    r = 30
    for i in range(center - r, center + r - 1):
        distance1 += ds[i] * abs(math.sin(math.radians(i - angle)))
    distance1 /= 2 * r
    distance1 %= 30
    center = round(angle + 2 * 90)
    distance2 = 0.0
    for i in range(center - r, center + r - 1):
        distance2 += ds[i] * abs(math.sin(math.radians(i - angle)))
    distance2 /= 2 * r
    distance2 %= 30
    return distance1, distance2


def fixAngle(angle):
    """Iteratively rotate to cancel the heading error, re-reading the LiDAR.

    Nudges the robot a little, re-measures the angle, and repeats until the error is
    within ``anglePrecision`` or three corrections have been made.
    """
    t = abs(round(angle * anglePrecision / 4))
    speed = 1000
    count = 0
    while abs(angle) > anglePrecision and count < 3:
        if angle > anglePrecision:
            motor.set_speeds([-speed, -speed, -speed, -speed])  # adjust right
        if angle < anglePrecision:
            motor.set_speeds([speed, speed, speed, speed])  # adjust left
        time.sleep_ms(t)

        dist = ld.read()
        ds = mins(dist)
        print(ds)
        angle = getAngle(ds)
        t = abs(round(angle * anglePrecision / 4))
        print(angle)
        count += 1
    motor.set_speeds([0, 0, 0, 0])


def checkWalls(angle, ds):
    """Return a 4-element wall bitmask from the distance scan ``ds``.

    For each of the four cardinal directions (relative to ``angle``), averages the
    projected distance over a +/-30-degree window and marks a wall when it is within
    the 250-unit threshold.
    """
    walls = [0, 0, 0, 0]
    for i in range(4):
        center = round(angle + i * 90)
        distance = 0.0
        r = 30
        for j in range(center - r, center + r - 1):
            distance += ds[j] * abs(math.sin(math.radians(j - angle)))
        distance /= 2 * r
        if distance <= 250:
            walls[(i + 1) % 4] = 1
    return walls


def move(d):
    """Drive one cell: ``d == 0`` forward, otherwise backward."""
    t = 3000
    speed = 2000
    if d == 0:
        motor.set_speeds([-speed, -speed, speed, speed])
    else:
        motor.set_speeds([speed, speed, -speed, -speed])
    time.sleep_ms(t)
    motor.set_speeds([0, 0, 0, 0])


# Quick drive test: move one cell forward.
move(0)
