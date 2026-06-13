"""Standalone LiDAR/ToF sensor diagnostic.

Brings up the two VL53L0X time-of-flight sensors on the I2C bus (using the XSHUT
pin to assign distinct addresses) plus the spinning Lidar, then continuously
prints the front, secondary, and downward distance readings. Used during
hardware bring-up and calibration, separate from the main navigation code.
"""

import pyb
from machine import I2C
from pyb2 import *
import time

# Hold the first sensor in reset so it powers up on its non-default address.
xshut = pyb.Pin('P6', pyb.Pin.OUT)
xshut.value(0)  # 0: shutdown, 1: power on
time.sleep_ms(50)

i2c = I2C(sda=pyb.Pin('P8'), scl=pyb.Pin('P7'), freq=400000)
sensor1 = VL53L0X(i2c, 0x2a)
xshut.value(1)
time.sleep_ms(50)
sensor2 = VL53L0X(i2c, 0x29)
sensor1.start()
sensor2.start()
lidar = Lidar(3)

while True:
    front = lidar.read()
    front2 = sensor1.read()
    down = sensor2.read()
    print("front dist:", front[0], front2, down)
