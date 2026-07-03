# SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
# Required Notice: Copyright (c) 2026 George Zhang — https://github.com/TheYellowDuck

# Navigation - By: gzhan - Fri May 12 2023

import sensor, image, time

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)

clock = time.clock()

while(True):
    clock.tick()
    img = sensor.snapshot()
    print(clock.fps())

# 0 = front/up
# 1 = right
# 2 = back/down
# 3 = left

o = 0
maze = {}
x = 0
y = 0

# 0 = blank tile
# -3 = black tile
# -2 = blue tile
# -1 = obstacle
# 1 = ramp
# 2 = checkpoint
# 3 = victim

def nav(f, b, l, r, t):
    maze[(x, y)] = (f,b,l,r,t)
    if t == -2:
        go(2)
        return
    if not getCrds(3) in maze:
        go(3)
    if not getCrds(0) in maze:
        go(0)
    if not getCrds(1) in maze:
        go(1)
    if not getCrds(2) in maze:
        go(2)

def goBack():
    pass



def go(d):
    if o == 0:
        if d == 0:
            y += 1
        if d == 1:
            x += 1
        if d == 2:
            y -= 1
        if d == 3:
            x -= 1
    if o == 1:
        if d == 0:
            x += 1
        if d == 1:
            y -= 1
        if d == 2:
            x -= 1
        if d == 3:
            y += 1
    if o == 2:
        if d == 0:
            y -= 1
        if d == 1:
            x -= 1
        if d == 2:
            y += 1
        if d == 3:
            x += 1
    if o == 3:
        if d == 0:
            x -= 1
        if d == 1:
            y += 1
        if d == 2:
            x += 1
        if d == 3:
            y -= 1

def getCrds(d):
    if o == 0:
        if d == 0:
            return (x, y + 1)
        if d == 1:
            return (x + 1, y)
        if d == 2:
            return (x, y - 1)
        if d == 3:
            return (x - 1, y)
    if o == 1:
        if d == 0:
            return (x + 1, y)
        if d == 1:
            return (x, y - 1)
        if d == 2:
            return (x - 1, y)
        if d == 3:
            return (x, y + 1)
    if o == 2:
        if d == 0:
            return (x, y - 1)
        if d == 1:
            return (x - 1, y)
        if d == 2:
            return (x, y + 1)
        if d == 3:
            return (x + 1, y)
    if o == 3:
        if d == 0:
            return (x - 1, y)
        if d == 1:
            teturn (x, y + 1)
        if d == 2:
            return (x + 1, y)
        if d == 3:
            return (x, y - 1)




