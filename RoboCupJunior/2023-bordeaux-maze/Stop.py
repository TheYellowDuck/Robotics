# SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
# Required Notice: Copyright (c) 2026 George Zhang — https://github.com/TheYellowDuck

"""Emergency stop: zero all four drive servos immediately.

Flashed to the robot to halt it when the main navigation program needs to be
killed quickly.
"""

from pyb2 import *

motor = SServo(1)
motor.set_speeds([0, 0, 0, 0])
