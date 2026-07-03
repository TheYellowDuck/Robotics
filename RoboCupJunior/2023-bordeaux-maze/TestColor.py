# SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
# Required Notice: Copyright (c) 2026 George Zhang — https://github.com/TheYellowDuck

import sensor, image, time
sensor.reset()
sensor.set_framesize(sensor.QVGA)
sensor.set_pixformat(sensor.RGB565)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_vflip(True)
sensor.set_hmirror(True)
clock = time.clock()
# Use the Tools -> Machine Vision -> Threshold Edtor to pick better thresholds.
red_threshold = (0, 100, 30-5,127, -128, 127)  # L A B
green_threshold = (0, 100, -128, -30+8, -128, 127)  # L A B
yellow_threshold = (0, 100, -128, 127, -128, 30-5)  # L A B
blue_threshold = (0, 100, -128, 127, -128, 0)  # L A B
# read first, yellow second, green third
while (True):
    for i in range(20):
        clock.tick()
        img = sensor.snapshot()
        print(clock.fps(), 'raw')
    # Test red threshold
    for i in range(10):
        clock.tick()
        img = sensor.snapshot()
        img.binary([red_threshold])
        print(clock.fps(), 'red')
    for i in range(10):
        clock.tick()
        img = sensor.snapshot()
        img.binary([green_threshold])
        print(clock.fps(), 'green')
    for i in range(10):
        clock.tick()
        img = sensor.snapshot()
        img.binary([yellow_threshold])
        print(clock.fps(), 'yellow')
