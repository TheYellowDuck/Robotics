# SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
# Required Notice: Copyright (c) 2026 George Zhang — https://github.com/TheYellowDuck

# Grayscale Binary Filter Example
#
# This script shows off the binary image filter. You may pass binary any
# number of thresholds to segment the image by.
import sensor, image, time
sensor.reset()
sensor.set_framesize(sensor.QVGA)
sensor.set_pixformat(sensor.RGB565)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.skip_frames(time=2000)
clock = time.clock()
low_threshold = (0, 60)
high_threshold = (100, 255)
bw=[(0, 20, -23, 25, -23, 25)]
while True:
    # Test low threshold
    for i in range(20):
        clock.tick()
        img = sensor.snapshot()
        # img.binary([low_threshold])
        print(clock.fps(), 'raw')
    # Test high threshold
    #for i in range(10):
        #clock.tick()
        #img = sensor.snapshot()
        #img.to_grayscale()
        #print(clock.fps(), 'grayscale')
    #for i in range(10):
        #clock.tick()
        #img = sensor.snapshot()
        #img.to_grayscale()
        #img.binary([low_threshold])
        #print(clock.fps(), 'bin')
    for i in range(10):
        clock.tick()
        img = sensor.snapshot()
        #img.to_grayscale()
        img.binary(bw)
        print(clock.fps(), 'th')
