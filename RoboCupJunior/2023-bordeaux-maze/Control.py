# Untitled - By: gzhan - Fri May 26 2023

from pyb2 import *
import time, math

ld = Lidar(3)
motor = SServo(1)
anglePrecision = 6.5

def getAngle(ds):
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
    speed = 1000
    count = 0
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
    #print(angle, t)
    time.sleep_ms(t)
    motor.set_speeds([0, 0, 0, 0])
    #pass

def fixPos(angle):
    center = round(angle)
    distance1 = 0.0
    r = 30
    for i in range(center - r, center + r - 1):
        #print(j)
        distance1 += ds[j] * abs(math.sin(math.radians(i - angle)))
    distance1 /= 2 * r
    distance1 %= 30
    center = round(angle + 2 * 90)
    distance2 = 0.0
    for i in range(center - r, center + r - 1):
        #print(j)
        distance2 += ds[j] * abs(math.sin(math.radians(i - angle)))
    distance2 /= 2 * r
    distance2 %= 30


def fixAngle(angle):
    t = abs(round(angle * anglePrecision / 4))
    speed = 1000
    count = 0
    while abs(angle) > anglePrecision and count < 3:
        if angle > anglePrecision:
            motor.set_speeds([-speed, -speed, -speed, -speed])
            #adjust right
        if angle < anglePrecision:
            #adjust left
            motor.set_speeds([speed, speed, speed, speed])
        time.sleep_ms(t)
        #t /= 2
        #t = round(t)
        #speed /= 2
        #speed = round(speed)
        #speed = round(angle * 100)

        dist = ld.read()
        ds = mins(dist)
        print(ds)
        angle = getAngle(ds)
        t = abs(round(angle * anglePrecision / 4))
        print(angle)
        count += 1
        #pass
    motor.set_speeds([0, 0, 0, 0])

def checkWalls(angle, ds):
    walls = [0,0,0,0]
    for i in range(4):
        center = round(angle + i * 90)
        distance = 0.0
        r = 30
        for j in range(center - r, center + r - 1):
            #print(j)
            distance += ds[j] * abs(math.sin(math.radians(j - angle)))
        distance /= 2 * r
        #print(distance)
        if distance <= 250:
            walls[(i + 1) % 4] = 1
    return walls

def move(d):
    t = 3000
    speed = 2000
    if d == 0:
        motor.set_speeds([-speed, -speed, speed, speed])
    else:
        motor.set_speeds([speed, speed, -speed, -speed])
    time.sleep_ms(t)
    motor.set_speeds([0, 0, 0, 0])



clock = time.clock()

#dist = ld.read()
#ds = mins(dist)
#print(ds)
#angle = getAngle(ds)
#print(angle)
#turn (angle, 1)
#dist = ld.read()
#ds = mins(dist)
#print(ds)
#angle = getAngle(ds)
#print(angle)
#if abs(angle) > anglePrecision:
    #fixAngle(angle)
#walls = checkWalls(angle, dist)
#print(walls)

move(0)

#fixAngle(angle)
#while True:
    #clock.tick()
    #dist = ld.read()
    ##for i in range(360):
        ##print(dist[i], end = ", ")
    ##print()
    ##print(dist.tolist())
    ##print(dist[90])
    #ds = mins(dist)
    #print(ds)
    #angle = getAngle(ds)
    #print(angle)
    ##getMinAngle(dist)
    ##if abs(angle) > anglePrecision:
        ##fixAngle(angle)
    #walls = checkWalls(angle, dist)
    #print(walls)
    #print()
    #time.sleep_ms(10)
    #print(clock.fps())
