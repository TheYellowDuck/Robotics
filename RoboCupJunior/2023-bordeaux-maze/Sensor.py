from pyb import millis, LED
import pyb
from machine import I2C, Pin
from pyb2 import *
import sensor, image, time, math
import sys
import gc

xshut = pyb.Pin('P6', pyb.Pin.OUT)
xshut.value(0) # 0:shutdown 1:power on
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
