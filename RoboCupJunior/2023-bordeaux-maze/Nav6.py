# SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
# Required Notice: Copyright (c) 2026 George Zhang — https://github.com/TheYellowDuck

"""Final maze-navigation program for the RoboCupJunior 2023 Maze robot.

Runs entirely on the OpenMV camera. Initialises the camera (QVGA, RGB565) and the
LAB colour thresholds for the maze tiles (red / green / yellow / blue / black /
white and letter victims), then runs the main perceive-decide-act loop: classify
the tile under the robot from the camera, read the LiDAR to detect walls on each
side, record each cell's walls as a 4-bit mask (forward / right / back / left), and
drive the motors accordingly.

This is the last of the iterative ``Nav`` rewrites (``Nav`` -> ``Nav6``); the
earlier versions are kept for history.
"""

from pyb import millis, LED
from machine import Pin
from pyb2 import *
import sensor, image, time, math
import sys

#SystemExit
sensor.reset()
sensor.set_framesize(sensor.QVGA)
sensor.set_pixformat(sensor.RGB565)
#sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.skip_frames(time=2000)

servo9 = PServo()
servo9.set(ms=1.5)

red_threshold = (0, 100, 8, 127, -18, 27)  # L A B
green_threshold = (0, 100, -128, -10, -15, 17)  # L A B
yellow_threshold = (0, 100, -27, -10, 24, 127)  # L A B
blue_threshold = (0, 100, -8, 9, -62, -15)  # L A B

#bw = [(0, 20, -23, 25, -23, 25)]
black_threshold = (9, 44, -8, 4, -10, 10)
white_threshold = (64, 100, -18, 5, -19, 5)

letter_threshold = (15, 60, -16, 3, -23, -5)

# from motor2 import *
clock = time.clock()
# abcdefg                     0 = no, 1 = yes
# a = forward wall
# b = right wall
# c = back wall
# d = left wall
# e = tile weight
# f = tile type
# g = tile specifics
''' API
motor2
    - port
    - turn right
    - turn left
    - turn back
    - forward ms
    - stop
lidar
    - port
    - read: timeout, percent
lidar data
    - set data
    - find lines
    - check walls
'''
''' API
motor2
    - port
    - turn right
    - turn left
    - turn back
    - forward ms
    - stop
lidar
    - port
    - read: timeout, percent
lidar data
    - set data
    - find lines
    - check walls
'''
''' API
motor2
    - port
    - turn right
    - turn left
    - turn back
    - forward ms
    - stop
lidar
    - port
    - read: timeout, percent
lidar data
    - set data
    - find lines
    - check walls
'''

# p1 = Pin("PG12", Pin.OUT)
# p2 = Pin("PC4", Pin.IN, Pin.PULL_UP)
p2 = Pin('P2', Pin.IN, Pin.PULL_UP)
p1 = Pin("P3", Pin.OUT)
button_pressed = 0
def button(p):
    v = p.value()
    if v == 1: return
    global button_pressed
    if button_pressed == 0:
        button_pressed = 1
        print('button_pressed', button_pressed)
        p1.value(1)
def proc_button():
    global button_pressed
    button_pressed = 0
    p1.value(0)
p2.irq(lambda p: button(p))
# use sts3215
# used by robot
class Motor:
    def __init__(self):
        print("create new motor")
        self.motor = SServo(1, [11, 12, 21, 22])
    def set_speeds(self, speeds):
        self.motor.set_speeds(speeds)
    def run(self, left, right):
        self.motor.set_speeds([-left, -left, right, right])
    def stop(self):
        self.motor.set_speeds([0, 0, 0, 0])
        return
        while True:
            if self.is_stopped():
                break
            time.sleep_ms(100)
    def is_stopped(self):
        # check servo for True or false
        r = self.motor.is_stopped()
        return r == 0
    # return distance in steps
    def forward_ms(self, speed, ms, check=lambda: False):
        t0 = millis()
        p0 = self.motor.get_position(2)
        c = 0
        last_p2 = 0
        p2 = last_p2
        flag = 0
        self.run(speed, speed)
        while True:
            t1 = millis()
            if t1 - t0 > ms: break
            if check(): break
            try:
                p1 = self.motor.get_position(2)
                p2 = (p1 - p0) % 4096
                if last_p2 == 0 and p2 > 4090:
                    p2 = 0
                    flag = 1
                if p2 < 2048 and last_p2 > 2048: c += 1
                last_p2 = p2
            except:
                pass
            # print('p1 p2 c', p1, p2, c)
        self.stop()
        s = p2 + 4096 * c
        # print('p0 p1 c s flag,', p0, p1, c, s,flag)
        return s
    def forward_cm(self, speed, cm):
        pass
    def forward_step(self, s=None, t=3000):
        if s is None:
            s = 3350 - 50 - 20
        self.motor.set_positions([-s, -s, s, s])
        time.sleep_ms(t)
        print('finished turn')
        return
    def turn_right(self, s=None, t=3000):
        if s is None:
            s = 3350 - 50 - 20 + 20
        self.motor.set_positions([-s, -s, -s, -s])
        time.sleep_ms(t)
        print('finished turn')
        return
        while True:
            time.sleep_ms(100)
            t = self.is_stopped()
            if t: break
    def turn_left(self, s=None, t=3000):
        if s is None:
            s = 3350 - 50 - 20 + 20
        self.motor.set_positions([s, s, s, s])
        time.sleep_ms(t)
        print('finished turn')
        return
        while True:
            time.sleep_ms(100)
            t = self.is_stopped()
            if t: break
    def turn_back(self, s=None, t=5000):
        if s is None:
            s = 7000 - 50 - 20
        self.motor.set_positions([s, s, s, s])
        time.sleep_ms(t)
        print('finished turn')
        return
    def backward_ms(self, speed, ms):
        pass
    def backward_cm(self, speed, cm):
        pass
    def drop_cube(self):
        #TODO: pull back stopper
        #servo9.set(ms = 1)
        time.sleep_ms(500)
        #TODO: push back stopper
        #servo9.set(ms = 2)
        print("Dropped Cube")
        flash(1)
        #pass
    def check_tile(self):
        #return tile type
        #1: stair
        #2: ramp
        cramp = robot.check_ramp()
        if cramp == 1:
            return 2
        if cramp == 2:
            return 3
        return 0
    def voltage(self):
        v = self.motor.voltage()
        return v

class Camera():
    def __init__(self):
        self.img = sensor.snapshot()
        self.tempCount = -1
        self.tempCount2 = -1

    def scan(self):
        self.img = sensor.snapshot()
        self.img.gaussian(1)

    def check_victims(self):
        # self.tempCount = (self.tempCount + 1) % 6
        # self.tempCount2 = (self.tempCount2 + 1) % 4
        #return victims [front,right,back,left]
        #10: G
        #11: Y
        #12: R
        #13: U
        #14: S
        #15: H
        self.scan()
        arr = [0,0,0,0]
        temp = [self.get_color_wall(), self.get_letter()]
        for i in range(4):
            for j in range(2):
                if temp[j][i] != 0:
                    arr[i] = temp[j][i]
                    break
        # arr[self.tempCount2] = 10 + self.tempCount
        return arr

    # def check_tile(self):
    #     self.scan()
    #     #1: white
    #     #2: silver
    #     #3: blue
    #     #4: black
    #     v = self.check_blue()
    #     if v == 1:
    #         v = self.check_black()
    #     return v

    # def check_red(self):
    #     filtered = self.img
    #     filtered.binary([red_threshold])
    #     return self.get_color_wall(filtered, 12)
    #     # pass
    # def check_yellow(self):
    #     filtered = self.img
    #     filtered.binary([yellow_threshold])
    #     return self.get_color_wall(filtered, 11)
    #     # pass
    # def check_green(self):
    #     filtered = self.img
    #     filtered.binary([green_threshold])
    #     return self.get_color_wall(filtered, 10)
    #     # pass
    # def check_blue(self):
    #     filtered = self.img
    #     filtered.binary([blue_threshold])
    #     return self.get_color_ground(filtered, 3)
    #     # pass
    # def check_black(self):
    #     filtered = self.img
    #     filtered.binary(bw)
    #     return self.get_color_ground(filtered, 4)
    #     # pass
    # def check_silver(self):
    #     pass

    def get_color_wall(self):
        self.scan()
        w = 320
        h = 240
        fov = 100

        arr = [0,0,0,0]

        robot.get_lidardata()

        self.img.draw_line(0, (int)(h/2) - 30, w, (int)(h/2) - 30)

        for blob in self.img.find_blobs([red_threshold, green_threshold, yellow_threshold], pixels_threshold=200, area_threshold=250, merge=False):
            if blob.y() >= (int) (h/2) - 30 and blob.area() <= 5000 and blob.h() >= blob.w() - 5 and blob.density() >= 0.5:
                # These values depend on the blob not being circular - otherwise they will be shaky.
                #if blob.elongation() > 0.5 and blob.elongation() < 0.9:
                    #self.img.draw_edges(blob.min_corners(), color=(255,0,0))
                    #self.img.draw_line(blob.major_axis_line(), color=(0,255,0))
                    #self.img.draw_line(blob.minor_axis_line(), color=(0,0,255))
                # These values are stable all the time.
                self.img.draw_rectangle(blob.rect())
                #self.img.draw_cross(blob.cx(), blob.cy())
                # Note - the blob rotation is unique to 0-180 only.
                #self.img.draw_keypoints([(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20)

                if blob.code() == 1:
                    self.img.draw_string(blob.x() + 2, blob.y() + 2, "red")
                if blob.code() == 2:
                    self.img.draw_string(blob.x() + 2, blob.y() + 2, "green")
                if blob.code() == 4:
                    self.img.draw_string(blob.x() + 2, blob.y() + 2, "yellow")

                bound = blob.rect()
                cx = blob.cx()
                cy = blob.cy()
                print(bound, cx, cy)

                blob_angle_to_robot = (cx - w/2.0)*fov/w

                blob_angle = robot.ld_ang + blob_angle_to_robot

                print(robot.ld_ang, blob_angle_to_robot, blob_angle)

                if -20 < blob_angle and blob_angle < 20 and blob.elongation() < 0.9:
                    if blob.code() == 1:
                        arr[0] = 12
                    if blob.code() == 2:
                        arr[0] = 10
                    if blob.code() == 4:
                        arr[0] = 11
                    #arr[0] = 1
                elif blob_angle <= -20 and blob.elongation() > 0.5 and blob.elongation() < 0.9:
                    if blob.code() == 1:
                        arr[3] = 12
                    if blob.code() == 2:
                        arr[3] = 10
                    if blob.code() == 4:
                        arr[3] = 11
                    #arr[3] = 1
                elif blob.elongation() > 0.5 and blob.elongation() < 0.9:
                    if blob.code() == 1:
                        arr[1] = 12
                    if blob.code() == 2:
                        arr[1] = 10
                    if blob.code() == 4:
                        arr[1] = 11
                    #arr[1] = 1
                #self.img.draw_rectangle((0, (int) (h/2) - 30, (int) (w/3), h - ((int) (h/2) - 30)))
                #self.img.draw_rectangle(((int) (w/3) - 25, (int) (h/2) - 30, (int) (w*2/3) + 25 - ((int) (w/3) - 25), h - 50 - ((int) (h/2) - 30)))
                #self.img.draw_rectangle(((int) (w*2/3), (int) (h/2) - 30, w - ((int) (w*2/3)), h - ((int) (h/2) - 30)))

                #if bound[0] >= 0 and bound[0] + bound[2] <= (int) (w/3) and bound[1] >= (int) (h/2) - 30 and bound[1] + bound[3] <= h:
                    #arr[3] = 1
                #if bound[0] >= (int) (w/3) - 25 and bound[0] + bound[2] <= (int) (w*2/3) + 25 and bound[1] >= (int) (h/2) - 30 and bound[1] + bound[3] <= h - 50:
                    #arr[0] = 1
                #if bound[0] >= (int) (w*2/3) and bound[0] + bound[2] <= w and bound[1] >= (int) (h/2) - 30 and bound[1] + bound[3] <= h:
                    #arr[1] = 1


            #pixelcount = 0
            #for i in range ((int)(h/2) - 10, (int) (h*4/5)):
                #for j in range (0, (int) (w/3)):
                    #pixel = self.img.get_pixel(j, i)
                    #if pixel[0] == 255:
                        #pixelcount += 1
            #print("Left: ", pixelcount)
            #if pixelcount > 300:
                #arr[3] = v

            #pixelcount = 0
            #for i in range ((int)(h/2) - 10, (int) (h*4/5)):
                #for j in range ((int) (w/3), (int) (w*2/3)):
                    #pixel = self.img.get_pixel(j, i)
                    #if pixel[0] == 255:
                        #pixelcount += 1
            #print("Front: ", pixelcount)
            #if pixelcount > 300:
                #arr[0] = v

            #pixelcount = 0
            #for i in range ((int)(h/2) - 10, (int) (h*4/5)):
                #for j in range ((int) (w*2/3), w):
                    #pixel = self.img.get_pixel(j, i)
                    #if pixel[0] == 255:
                        #pixelcount += 1
            #print("Right: ", pixelcount)
            #if pixelcount > 300:
                #arr[1] = v

        return arr

    def get_color_ground(self):
        self.scan()

        w = 320
        h = 240

        robot.get_lidardata()

        #self.img.draw_rectangle(((int) (w/8), (int) (h/2) + 35, (int) (w*6/8), h - ((int)(h/2) - 35)))

        for blob in self.img.find_blobs([white_threshold, blue_threshold, black_threshold], pixels_threshold=1500, area_threshold=3000, merge=False, roi = ((int) (w/8), (int) (h/2) + 35, (int)(w*6/8), h - ((int) (h/2) + 35))):
            if blob.y() >= (int) (h/2):
                # These values depend on the blob not being circular - otherwise they will be shaky.
                #if blob.elongation() > 0.5:
                    #self.img.draw_edges(blob.min_corners(), color=(255,0,0))
                    #self.img.draw_line(blob.major_axis_line(), color=(0,255,0))
                    #self.img.draw_line(blob.minor_axis_line(), color=(0,0,255))
                # These values are stable all the time.
                self.img.draw_rectangle(blob.rect())
                #self.img.draw_cross(blob.cx(), blob.cy())
                # Note - the blob rotation is unique to 0-180 only.
                #self.img.draw_keypoints([(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20)

                if blob.code() == 1:
                    self.img.draw_string(blob.x() + 2, blob.y() + 2, "white")
                if blob.code() == 2:
                    self.img.draw_string(blob.x() + 2, blob.y() + 2, "blue")
                if blob.code() == 4:
                    self.img.draw_string(blob.x() + 2, blob.y() + 2, "black")

                bound = blob.rect()

                #if bound[0] >= (int) (w/8) and bound[0] + bound[2] <= (int) (w*7/8):
                if blob.code() == 1:
                    return 1
                if blob.code() == 2:
                    return 3
                if blob.code() == 4:
                    return 4

        #w = 320
        #h = 240

        #pixelcount = 0
        #for i in range ((int)(h*2/3) + 5, (int)(h*2/3) + 20):
            #for j in range (30, 290):
                #pixel = self.img.get_pixel(j, i)
                #if pixel[0] == 255:
                    #pixelcount += 1
        #if pixelcount > 3000:
            #return v

        return 2

    def check_H(self):
        pass
    def check_S(self):
        pass
    def check_U(self):
        pass

    def check_threshold(self, pixel):
        for i in range(6):
            if i % 2 == 0 and letter_threshold[i] > pixel[(int) (i / 2)]:
                return 0
            if i % 2 == 1 and letter_threshold[i] < pixel[(int) (i / 2)]:
                return 0
        return 1

    def check_letter(self, blob):
        print("Text Blob", blob.pixels(), blob.area())
        pixelcount = 0
        for i in range(blob.cy() - 5, blob.cy() + 10):
            pixel = self.img.get_pixel(blob.cx(), i)
            pixel = image.rgb_to_lab(pixel)
            if self.check_threshold(pixel) == 1:
                pixelcount += 1
            #print(pixel)
        #time.sleep_ms(1000)
        #for i in range(blob.cy() - 5, blob.cy() + 10):
            #self.img.draw_rectangle(blob.cx(), i, 1, 1)
        if pixelcount < 2:
            #if blob.elongation() < 0.9:
            self.img.draw_string(blob.x() + 2, blob.y() + 2, "U")
            return 13
        pixelcount = 0
        for i in range(blob.y(), blob.y() + 10):
            pixel = self.img.get_pixel(blob.cx(), i)
            pixel = image.rgb_to_lab(pixel)
            if self.check_threshold(pixel) == 1:
                pixelcount += 1
            #print(pixel)
        #time.sleep_ms(1000)
        if pixelcount < 2:
            #if blob.elongation() < 0.9:
            self.img.draw_string(blob.x() + 2, blob.y() + 2, "H")
            return 15
        #if blob.elongation() < 0.9:
        self.img.draw_string(blob.x() + 2, blob.y() + 2, "S")
        return 14

    def get_letter(self):
        self.scan()

        w = 320
        h = 240
        fov = 100

        arr = [0,0,0,0]

        robot.get_lidardata()

        #self.img.binary([letter_threshold])

        #self.img.draw_rectangle((0, (int) (h/2) - 30, w, (int)(h/3) + 10))

        for blob in self.img.find_blobs([letter_threshold], pixels_threshold=10, area_threshold=10, merge=True, roi = (0, (int) (h/2) - 30, w, (int)(h/3) + 10)):
            if blob.area() <= 5000 and blob.h() >= blob.w() - 5 and blob.density() < 0.5:
                # These values depend on the blob not being circular - otherwise they will be shaky.
                #if blob.elongation() > 0.5 and blob.elongation() < 0.9:
                    #self.img.draw_edges(blob.min_corners(), color=(255,0,0))
                    #self.img.draw_line(blob.major_axis_line(), color=(0,255,0))
                    #self.img.draw_line(blob.minor_axis_line(), color=(0,0,255))
                # These values are stable all the time.
                if blob.elongation() < 0.9:
                    self.img.draw_rectangle((blob.x() - 5, blob.y() - 5, blob.w() + 10, blob.h() + 10))
                #self.img.draw_cross(blob.cx(), blob.cy(), color=(255,255,0))
                # Note - the blob rotation is unique to 0-180 only.
                #self.img.draw_keypoints([(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20, color=(255,255,0))

                bound = blob.rect()
                cx = blob.cx()
                cy = blob.cy()
                print(bound, cx, cy)

                blob_angle_to_robot = (cx - w/2.0)*fov/w

                blob_angle = robot.ld_ang + blob_angle_to_robot

                print(robot.ld_ang, blob_angle_to_robot, blob_angle)

                if -20 < blob_angle and blob_angle < 20 and blob.elongation() < 0.9:
                    #if blob.code() == 1:
                        #arr[0] = 12
                    #if blob.code() == 2:
                        #arr[0] = 10
                    #if blob.code() == 4:
                        #arr[0] = 11
                    arr[0] = self.check_letter(blob)
                elif blob_angle <= -20 and blob.elongation() > 0.5 and blob.elongation() < 0.9:
                    #if blob.code() == 1:
                        #arr[3] = 12
                    #if blob.code() == 2:
                        #arr[3] = 10
                    #if blob.code() == 4:
                        #arr[3] = 11
                    arr[3] = self.check_letter(blob)
                elif blob.elongation() > 0.5 and blob.elongation() < 0.9:
                    #if blob.code() == 1:
                        #arr[1] = 12
                    #if blob.code() == 2:
                        #arr[1] = 10
                    #if blob.code() == 4:
                        #arr[1] = 11
                    arr[1] = self.check_letter(blob)
                #self.img.draw_rectangle((0, (int) (h/2) - 30, (int) (w/3), h - ((int) (h/2) - 30)))
                #self.img.draw_rectangle(((int) (w/3) - 25, (int) (h/2) - 30, (int) (w*2/3) + 25 - ((int) (w/3) - 25), h - 50 - ((int) (h/2) - 30)))
                #self.img.draw_rectangle(((int) (w*2/3), (int) (h/2) - 30, w - ((int) (w*2/3)), h - ((int) (h/2) - 30)))

                #if bound[0] >= 0 and bound[0] + bound[2] <= (int) (w/3) and bound[1] >= (int) (h/2) - 30 and bound[1] + bound[3] <= h:
                    #arr[3] = 1
                #if bound[0] >= (int) (w/3) - 25 and bound[0] + bound[2] <= (int) (w*2/3) + 25 and bound[1] >= (int) (h/2) - 30 and bound[1] + bound[3] <= h - 50:
                    #arr[0] = 1
                #if bound[0] >= (int) (w*2/3) and bound[0] + bound[2] <= w and bound[1] >= (int) (h/2) - 30 and bound[1] + bound[3] <= h:
                    #arr[1] = 1

        #filtered = self.img
        #filtered.binary(bw)
        #return self.get_letter_wall(filtered)

        return arr

class Robot():
    def __init__(self):
        self.ld_ang = 0
        self.motor = Motor()
        self.cam = Camera()
        self.lidar = Lidar(3)
        self.lidar_data = LidarData([])
        self.ldata = []
        self.rampDir = -1
        self.wall_ref = 150
        v = self.motor.voltage()
        print('voltage: ', v)
    def check_wall_reference(self):
        data = self.lidar.read()
        self.lidar_data.data = data
        ds = self.lidar_data.get_distance([0, 90, 180, 270])
        print(ds)
        m = min(ds)
        self.wall_ref = m
    def check_walls(self):
        self.get_lidardata()
        #print(self.ck_walls)
        return self.ck_walls[0]
    def arr_eql(self, a, b):
        for i in range(len(a)):
            if a[i] != b[i]:
                return 0
        return 1
    def check_tile_info(self):
        rtile = 1
        rvictims = [0]*4
        tile = [0]*3
        victims = [[0]*4]*3
        for i in range(3):
            tile[i] = self.motor.check_tile() + 4
            if tile[i] == 4:
                tile[i] = 1
                #tile[i] = self.cam.get_color_ground()
            #victims[i] = self.cam.check_victims()
        rtile = tile[0]
        if rtile != tile[1] and rtile != tile[2] and tile[1] == tile[2]:
            rtile = tile[1]
        rvictims = victims[0]
        for i in range(4):
            if rvictims[i] != victims[1][i] and rvictims[i] != victims[2][i] and victims[1][i] == victims[2][i]:
                rvictims[i] = victims[1][i]
        #if self.arr_eql(rvictims, victims[1]) == 1 and self.arr_eql(rvictims, victims[2]) == 1 and self.arr_eql(victims[1], victims[2]) == 1:
            #rvictims = victims[1]
        if rtile == 4:
            rvictims = [0,0,0,0]
        if rtile == 5:
            rvictims = [0,0,0,0]
        if rtile == 6:
            rvictims = [0,0,0,0]
        return [rvictims, rtile]
        #pass
    # def forward_tile(self):
    #     self.motor.forward_step(s=6500)
    #     time.sleep_ms(3000)
    def check_front(self):
        self.get_lidardata()
        print('front ', self.lidar_data.data[0])
        if self.lidar_data.data[0] < self.wall_ref:
            return True
        return False
    def forward(self):

        #check victim
        v = navi.tileInput()
        if v == -1:
            return -1

        self.get_lidardata()
        # print(self.ck_walls)
        if robot.rampDir == 1:
            startT = millis()
            self.motor.run(2500, 2500)
            time.sleep_ms(3150)
            self.get_lidardata()
            front = self.lidar_data.data[0]
            while front > 1500:
                self.get_lidardata()
                front = self.lidar_data.data[0]
            time.sleep_ms(500)
            self.motor.stop()
            time.sleep_ms(200)
            timePassed = millis() - startT
            cellsTraveled = (int) (timePassed / 3250)
            print("TIME: ", timePassed, cellsTraveled)
            for i in range(cellsTraveled - 2):
                if navi.path:
                    navi.path.pop(0)
                if navi.o == 0:
                    navi.y += 1
                if navi.o == 1:
                    navi.x += 1
                if navi.o == 2:
                    navi.y -= 1
                if navi.o == 3:
                    navi.x -= 1
                print("------ ", navi.x, navi.y, navi.z)
                data = [0,0,0,0,2,1,0]
                #data[(navi.o + 2) % 4] = (int)(navi.z / 10) + 4
                data[(navi.o + 1) % 4] = 1
                data[(navi.o + 3) % 4] = 1
                maz.maze[(navi.x, navi.y, navi.z)] = data
            if navi.path:
                navi.path.pop(0)
            robot.rampDir = -1
            maz.maze[(navi.x, navi.y, navi.z)][navi.o] = (int)((navi.z + 5) / 10) + 4
            if navi.o == 0:
                navi.y += 1
            if navi.o == 1:
                navi.x += 1
            if navi.o == 2:
                navi.y -= 1
            if navi.o == 3:
                navi.x -= 1
            navi.z += 5
            #self.motor.forward_ms(2500, 2900, self.check_front)
            #self.get_lidardata()
            #front = self.lidar_data.data[0]
            #while front > 1500:
                #if navi.o == 0:
                    #navi.y += 1
                #if navi.o == 1:
                    #navi.x += 1
                #if navi.o == 2:
                    #navi.y -= 1
                #if navi.o == 3:
                    #navi.x -= 1
                #self.motor.forward_ms(2500, 2900, self.check_front)
                #print("------ ", navi.x, navi.y, navi.z)
                #self.get_lidardata()
                #front = self.lidar_data.data[0]
                #if front > 1500:
                    #data = [0,0,0,0,2,1,0]
                    ##data[(navi.o + 2) % 4] = (int)(navi.z / 10) + 4
                    #data[(navi.o + 1) % 4] = 1
                    #data[(navi.o + 3) % 4] = 1
                    #maz.maze[(navi.x, navi.y, navi.z)] = data
                #else:
                    #if navi.o == 0:
                        #navi.y -= 1
                    #if navi.o == 1:
                        #navi.x -= 1
                    #if navi.o == 2:
                        #navi.y += 1
                    #if navi.o == 3:
                        #navi.x += 1
                    #robot.rampDir = -1
                    #maz.maze[(navi.x, navi.y, navi.z)][navi.o] = (int)((navi.z + 5) / 10) + 4
                    #if navi.o == 0:
                        #navi.y += 1
                    #if navi.o == 1:
                        #navi.x += 1
                    #if navi.o == 2:
                        #navi.y -= 1
                    #if navi.o == 3:
                        #navi.x -= 1
                    #navi.z += 5
        elif robot.rampDir == 0:
            startT = millis()
            self.motor.run(2000, 2000)
            time.sleep_ms(3150)
            self.get_lidardata()
            back = self.lidar_data.data[180]
            while back > 1500:
                self.get_lidardata()
                back = self.lidar_data.data[180]
            time.sleep_ms(500)
            self.motor.stop()
            time.sleep_ms(200)
            timePassed = millis() - startT
            cellsTraveled = (int) (timePassed / 3250)
            print("TIME: ", timePassed, cellsTraveled)
            for i in range(cellsTraveled - 2):
                if navi.path:
                    navi.path.pop(0)
                if navi.o == 0:
                    navi.y += 1
                if navi.o == 1:
                    navi.x += 1
                if navi.o == 2:
                    navi.y -= 1
                if navi.o == 3:
                    navi.x -= 1
                print("------ ", navi.x, navi.y, navi.z)
                data = [0,0,0,0,2,1,0]
                #data[(navi.o + 2) % 4] = (int)(navi.z / 10) + 4
                data[(navi.o + 1) % 4] = 1
                data[(navi.o + 3) % 4] = 1
                maz.maze[(navi.x, navi.y, navi.z)] = data
            if navi.path:
                navi.path.pop(0)
            robot.rampDir = -1
            maz.maze[(navi.x, navi.y, navi.z)][navi.o] = (int)((navi.z - 5) / 10) + 4
            if navi.o == 0:
                navi.y += 1
            if navi.o == 1:
                navi.x += 1
            if navi.o == 2:
                navi.y -= 1
            if navi.o == 3:
                navi.x -= 1
            navi.z -= 5
            #self.motor.forward_ms(2500, 1800, self.check_front)
            #self.get_lidardata()
            #back = self.lidar_data.data[180]
            #while back > 1500:
                #if navi.o == 0:
                    #navi.y += 1
                #if navi.o == 1:
                    #navi.x += 1
                #if navi.o == 2:
                    #navi.y -= 1
                #if navi.o == 3:
                    #navi.x -= 1
                #self.motor.forward_ms(2500, 1800, self.check_front)
                #print("------ ", navi.x, navi.y, navi.z)
                #self.get_lidardata()
                #back = self.lidar_data.data[180]
                #if back > 1500:
                    #data = [0,0,0,0,2,1,0]
                    ##data[(navi.o + 2) % 4] = (int)(navi.z / 10) + 4
                    #data[(navi.o + 1) % 4] = 1
                    #data[(navi.o + 3) % 4] = 1
                    #maz.maze[(navi.x, navi.y, navi.z)] = data
                #else:
                    #if navi.o == 0:
                        #navi.y -= 1
                    #if navi.o == 1:
                        #navi.x -= 1
                    #if navi.o == 2:
                        #navi.y += 1
                    #if navi.o == 3:
                        #navi.x += 1
                    #robot.rampDir = -1
                    #maz.maze[(navi.x, navi.y, navi.z)][navi.o] =  (int)((navi.z - 5) / 10) + 4
                    #if navi.o == 0:
                        #navi.y += 1
                    #if navi.o == 1:
                        #navi.x += 1
                    #if navi.o == 2:
                        #navi.y -= 1
                    #if navi.o == 3:
                        #navi.x -= 1
                    #navi.z -= 5
        else:
            #if self.ck_walls[0][2] == 1 or self.ck_walls[1][2][1] == 1:
                ## self.forward_reference_back_dist()
                #self.motor.forward_ms(2500, 2400, self.check_front)
            #else:
                #self.motor.forward_ms(2500, 2400, self.check_front)
            self.motor.forward_ms(2500, 2400, self.check_front)
        self.adjust_angle()
        self.adjust_to_front()
        self.adjust_left_and_right()
        return 1
    def shift_pos(self, d, t, c):
        self.motor.run(d, -d)
        time.sleep_ms(400)
        self.motor.stop()
        time.sleep_ms(200)
        self.motor.run(2000, 2000)
        time.sleep_ms(t)
        self.motor.stop()
        time.sleep_ms(200)
        self.motor.run(-d, d)
        time.sleep_ms(800)
        self.motor.stop()
        time.sleep_ms(200)
        self.motor.run(-2000, -2000)
        time.sleep_ms(t)
        self.motor.stop()
        time.sleep_ms(200)
        self.motor.run(d, -d)
        time.sleep_ms(400)
        self.motor.stop()
        time.sleep_ms(200)
        if c == 2:
            self.motor.run(d, -d)
            time.sleep_ms(400)
            self.motor.stop()
            time.sleep_ms(200)
            self.motor.run(2000, 2000)
            time.sleep_ms((int) (t / 2))
            self.motor.stop()
            time.sleep_ms(200)
            self.motor.run(-d, d)
            time.sleep_ms(800)
            self.motor.stop()
            time.sleep_ms(200)
            self.motor.run(-2000, -2000)
            time.sleep_ms((int) (t / 2))
            self.motor.stop()
            time.sleep_ms(200)
            self.motor.run(d, -d)
            time.sleep_ms(400)
            self.motor.stop()
            time.sleep_ms(200)
        #for i in range(t):
            #self.motor.run(d, -d)
            #time.sleep_ms(400)
            #self.motor.stop()
            #time.sleep_ms(200)
            #self.motor.run(1000, 1000)
            #time.sleep_ms(400)
            #self.motor.stop()
            #time.sleep_ms(200)
            #self.motor.run(-d, d)
            #time.sleep_ms(400)
            #self.motor.stop()
            #time.sleep_ms(200)
            #self.motor.run(-1000, -1000)
            #time.sleep_ms(400)
            #self.motor.stop()
            #time.sleep_ms(200)
        #self.motor.run(d, -d)
        #time.sleep_ms(400)
        #self.motor.stop()
        #time.sleep_ms(200)
        #self.motor.run(1000, 1000)
        #time.sleep_ms(400)
        #self.motor.stop()
        #time.sleep_ms(200)
        #self.motor.run(-d, d)
        #time.sleep_ms(200)
        #self.motor.stop()
        #time.sleep_ms(200)
        #self.motor.stop()
    def adjust_left_and_right(self):
        self.get_lidardata()
        d1 = self.lidar_data.get_distance([90])[0]
        d2 = self.lidar_data.get_distance([270])[0]
        front = self.lidar_data.data[0]

        print(d1, d2)
        if d1 < 100:
            if front < 200:
                self.shift_pos(-1000, (int) ((150 - d1) * 7.5), 2)
            else:
                self.shift_pos(-1000, (int) ((150 - d1) * 10), 1)
        if d2 < 100:
            if front < 200:
                self.shift_pos(1000, (int) ((150 - d2) * 7.5), 2)
            else:
                self.shift_pos(1000, (int) ((150 - d2) * 10), 1)

        #pass
    def adjust_to_front(self):
        self.get_lidardata()
        d = self.lidar_data.get_distance([0])[0]
        print('ref d', self.wall_ref, d)
        #s = d - self.wall_ref
        if d < 100:
            self.motor.run(-2000, -2000)
            time.sleep_ms((int)((150 - d) * 8))
            self.motor.stop()
            time.sleep_ms(200)
        if d < 300 and d > 120:
        #if d > 120:
            self.motor.run(2000, 2000)
            time.sleep_ms((int)(((d % 300) - 120) * 10))
            self.motor.stop()
            time.sleep_ms(200)

            #s = d - self.wall_ref
            #speed = 1000
            #if s < 0: speed = -1000
            #if s < -20 or s > 20:
                #self.motor.run(speed, speed)
                #time.sleep_ms(300)
                #self.motor.stop()
    def adjust_to_back(self):
        self.get_lidardata()
        d = self.lidar_data.get_distance([180])[0]
        print('ref d', self.wall_ref, d)
        if d < 250:
            s = d - self.wall_ref
            speed = -1000
            if s > 0: speed = -1000
            if s < -20 or s > 20:
                self.motor.run(speed, speed)
                time.sleep_ms(300)
                self.motor.stop()
    def adjust_angle(self):
        self.get_lidardata()
        print(self.ld_lines)
        print(self.ld_ang)
        aa = self.lidar_data.find_angle()
        print('aa', aa)
        for i in range(len(aa)):
            a = aa[i]
            if math.fabs(a) < 20:
                aa[i] = a
            elif math.fabs(a - 90) < 20:
                aa[i] = a - 90
            elif math.fabs(a - 180) < 20:
                aa[i] = a - 180
            elif math.fabs(a - 270) < 20:
                aa[i] = a - 270
        # self.ld_ang=0
        print('aa', aa)
        if len(aa) == 1:
            self.ld_ang = aa[0]
        elif len(aa) > 1:
            self.ld_ang = np.mean(aa)
        else:
            self.ld_ang = 0
        print('ld ang', self.ld_ang)
        if abs(self.ld_ang) <= 5:  # change to 3 from 5
            return
        if self.ld_ang < 0:
            self.motor.run(-1000, 1000)
        else:
            self.motor.run(1000, -1000)
        if abs(self.ld_ang) <= 7.5:
            time.sleep_ms(300)
        elif abs(self.ld_ang) <= 10:
            time.sleep_ms(400)
        elif abs(self.ld_ang) <= 12.5:
            time.sleep_ms(500)
        elif abs(self.ld_ang) <= 15:
            time.sleep_ms(600)
        elif abs(self.ld_ang) <= 17.5:
            time.sleep_ms(650)
        elif abs(self.ld_ang) <= 20:
            time.sleep_ms(675)
        elif abs(self.ld_ang) <= 22.5:
            time.sleep_ms(700)
        elif abs(self.ld_ang) <= 25:
            time.sleep_ms(725)
        else:
            self.motor.stop()
            flash_red(5)
            SystemExit
        self.motor.stop()
        time.sleep_ms(200)
    def right(self):
        self.motor.turn_right()
        self.adjust_angle()
        self.adjust_to_back()
        self.adjust_left_and_right()
        return self.forward()
    def back(self):
        self.motor.turn_back()
        self.adjust_angle()
        self.adjust_to_back()
        self.adjust_left_and_right()
        return self.forward()
    def left(self):
        self.motor.turn_left()
        self.adjust_angle()
        self.adjust_to_back()
        self.adjust_left_and_right()
        return self.forward()
    def get_lidardata(self):
        self.ldata = self.lidar.read(timeout=3000, percent=100)
        self.ldata = np.array(self.ldata, dtype=np.float)
        self.lidar_data.data = self.ldata
        self.ck_walls = self.lidar_data.check_walls()
        # print(self.ck_walls)
        self.ld_lines = self.lidar_data.find_lines()
        # print(self.ld_lines)
        cnt = 0
        sum = 0
        self.ld_ang = 0
        for m, v in self.ld_lines:
            if v < 5:
                mm = m
                if m < -45:
                    mm += 90
                elif m > 45:
                    mm -= 90
                # print(m, mm)
                cnt += 1
                sum += mm
            # else:
            #     print('bad data', m, v)
        if cnt > 0:
            self.ld_ang = sum / cnt
        else:
            self.ld_ang = 0
    def forward_reference_back_dist(self):
        self.stop_back_dist = 350
        self.act_stop_back_dist = 400
        if self.ck_walls[0][2] == 0:
            self.stop_back_dist = 350 + 250
            self.act_stop_back_dist = 400 + 250 + 30
        print('ang', self.ld_ang)
        back_ds = self.ldata[180]
        print('bk ds', back_ds)
        while back_ds < self.stop_back_dist:
            self.motor.run(2000, 2000)
            # todo: adjust based on distance to left
            self.get_lidardata()
            print('ang', self.ld_ang)
            back_ds = self.ldata[180]
            print('bk ds', back_ds)
        print(self.act_stop_back_dist - back_ds)
        ts = (self.act_stop_back_dist - back_ds) * 6
        time.sleep_ms(int(ts))
        self.motor.stop()
    def check_ramp(self):
        #TODO check_ramp
        self.get_lidardata()
        front = self.lidar_data.data[0]
        right = self.lidar_data.data[20]
        left = self.lidar_data.data[340]
        print("front dist:", front, navi.previous_r)
        if front > 240 and front < 360:
            print("ramp")
            maz.ramps.append((navi.x, navi.y, navi.z))
            return 1
        if navi.previous_r == -1 and front > 2000 and (right > 2000 or left > 2000):
            print("ramp")
            maz.ramps.append((navi.x, navi.y, navi.z))
            return 2
        #robot.rampDir = -10
        return
    def drop_cube(self, d, count):
        if d == 0:
            flash()
            for i in range(count):
                self.motor.drop_cube()
        if d == 1:
            self.motor.turn_right()
            flash()
            for i in range(count):
                self.motor.drop_cube()
            self.motor.turn_left()
        if d == 2:
            self.motor.turn_back()
            flash()
            for i in range(count):
                self.motor.drop_cube()
            self.motor.turn_back()
        if d == 3:
            self.motor.turn_left()
            flash()
            time.sleep_ms(1500)
            for i in range(count):
                self.motor.drop_cube()
            self.motor.turn_right()
        self.adjust_angle()
        self.adjust_to_back()
        self.adjust_left_and_right()
        print("Finished Dropping")
        #time.sleep_ms(2000)
    def go(self, d):
        if (d == 0):  # go forwards
            return self.forward()
        if (d == 1):  # go right
            return self.right()
        if (d == 2):  # go back
            return self.back()
        if (d == 3):  # go left
            return self.left()
    def test(self):
        # task 1: create a robot that can go forward for 1 tile, also turn 90 deg left right
        # self.motor.forward_ms(2500, 2400)
        # self.motor.turn_left()
        # time.sleep_ms(3000)
        # self.motor.turn_back()
        # time.sleep_ms(3000)
        # self.motor.turn_right()
        # task 2: test data from lidardata
        # self.ldata = self.lidar.read(timeout=3000, percent=100)
        # self.ldata = np.array(self.ldata, dtype=np.float)
        #
        # self.lidar_data.data = self.ldata
        # ck_walls = self.lidar_data.check_walls()
        # print(ck_walls)
        # ld_lines = self.lidar_data.find_lines()
        # print(ld_lines)
        # task 3: follow wall to next tile
        # self.get_lidardata()
        # if self.ck_walls[0][0] == 0 and self.ck_walls[0][3] == 1 and self.ck_walls[1][0][0] == 1:
        #     if self.ck_walls[0][2] == 1 or self.ck_walls[1][2][1] == 1:
        #         self.stop_back_dist = 350
        #         self.act_stop_back_dist = 400
        #         if self.ck_walls[0][2] == 0:
        #             self.stop_back_dist = 350 + 250
        #             self.act_stop_back_dist = 400 + 250 + 30
        #         print("go")
        #         self.follow_left_wall()
        #     else:
        #         self.motor.stop()
        #         print("stop")
        # else:
        #     self.motor.stop()
        #     print("stop")
        # task 4: implement one function to control robot
        # print(robot.motor.voltage())
        # robot.go(0)
        # time.sleep_ms(1000)
        # robot.go(1)
        # time.sleep_ms(1000)
        # robot.go(2)
        # time.sleep_ms(1000)
        # robot.go(3)
        # time.sleep_ms(1000)
        # task 5: add an input function for nav
        pass
class Nav:
    def __init__(self):
        #         self.maze = {}
        #         self.o = 0
        #         self.x = 0
        #         self.y = 0
        self.x = 0
        self.y = 0
        self.z = 0
        self.o = 0
        self.ramp = -1
        self.step = 0
        self.prev = 0
        self.prevT = (0, 0, 0)
        self.previous_r = -1
        # self.count = 0
        self.path = []
    def Input(self):
        if (self.x, self.y, self.z) in maz.maze:
            input = maz.maze[(self.x, self.y, self.z)].copy()
            if self.o == 1:
                temp = input[0]
                input[0] = input[1]
                input[1] = input[2]
                input[2] = input[3]
                input[3] = temp
            if self.o == 2:
                temp = input[0]
                input[0] = input[2]
                input[2] = temp
                temp = input[1]
                input[1] = input[3]
                input[3] = temp
            if self.o == 3:
                temp = input[0]
                input[0] = input[3]
                input[3] = input[2]
                input[2] = input[1]
                input[1] = temp
            #if input[5] == 1:
                #if self.z < input[6]:
                    #input[6] = 1
                #elif self.z > input[6]:
                    #input[6] = 0
                #else:
                    #input[6] = 0
                    #robot.rampDir = -1
            #else:
                #input[6] = 0
                #robot.rampDir = -1
            #if input[5] == 1:
                #robot.rampDir = input[6]
            #else:
                #robot.rampDir = -1
            return input
        input = robot.check_walls()
        tcolor = 1
        if (self.x, self.y, self.z) in maz.coloured:
            tcolor = maz.coloured[(self.x, self.y, self.z)]
        if tcolor == 1:
            input.append(1)
        if tcolor == 2:
            input.append(1)
        if tcolor == 3:
            input.append(3)
        if tcolor == 4:
            input.append(999)
        if len(input) == 5:
            input.append(0)
        if len(input) == 6:
            input.append(0)
        #if (self.x, self.y, self.z) in maz.maze:
            #input = maz.maze[(self.x, self.y, self.z)]
            ##input[4] = data[4]
            ##input[5] = data[5]
            ##input[6] = data[6]
        #if input[5] == 1:
            #if self.Z < input[6]:
                #input[6] = 1
            #elif self.Z > input[6]:
                #input[6] = 0
            #else:
                #input[6] = 0
                #robot.rampDir = -1
        #else:
            #input[6] = 0
            #robot.rampDir = -1
        #if input[5] == 1:
            #robot.rampDir = input[6]
        #else:
            #robot.rampDir = -1
        return input

    def tileInput(self):
        nextTileInfo = robot.check_tile_info()
        if robot.ck_walls[1][0][0] == 0:
            nextTileInfo[0][3] = 0
        if robot.ck_walls[1][0][1] == 0:
            nextTileInfo[0][0] = 0
        if robot.ck_walls[1][0][2] == 0:
            nextTileInfo[0][1] = 0
        print("----- ", nextTileInfo, " -----")
        if nextTileInfo[1] == 1:
            victimInfo = maz.save(nextTileInfo[0])
            print(victimInfo)

            if (self.x, self.y, self.z) in maz.victims:
                pastVictimInfo = maz.victims[(self.x, self.y, self.z)]
                for i in range(4):
                    if pastVictimInfo[i] == 0:
                        pastVictimInfo[i] = victimInfo[i]
                victimInfo = pastVictimInfo
            maz.victims[(self.x, self.y, self.z)] = victimInfo

            #if self.o == 0:
                #if (self.x, self.y + 1, self.z) in maz.victims:
                    #pastVictimInfo = maz.victims[(self.x, self.y + 1, self.z)]
                    #for i in range(4):
                        #if pastVictimInfo[i] == 0:
                            #pastVictimInfo[i] = victimInfo[i]
                    #victimInfo = pastVictimInfo
                #maz.victims[(self.x, self.y + 1, self.z)] = victimInfo
            #if self.o == 1:
                #if (self.x + 1, self.y, self.z) in maz.victims:
                    #pastVictimInfo = maz.victims[(self.x + 1, self.y, self.z)]
                    #for i in range(4):
                        #if pastVictimInfo[i] == 0:
                            #pastVictimInfo[i] = victimInfo[i]
                    #victimInfo = pastVictimInfo
                #maz.victims[(self.x + 1, self.y, self.z)] = victimInfo
            #if self.o == 2:
                #if (self.x, self.y - 1, self.z) in maz.victims:
                    #pastVictimInfo = maz.victims[(self.x, self.y - 1, self.z)]
                    #for i in range(4):
                        #if pastVictimInfo[i] == 0:
                            #pastVictimInfo[i] = victimInfo[i]
                    #victimInfo = pastVictimInfo
                #maz.victims[(self.x, self.y - 1, self.z)] = victimInfo
            #if self.o == 3:
                #if (self.x - 1, self.y, self.z) in maz.victims:
                    #pastVictimInfo = maz.victims[(self.x - 1, self.y, self.z)]
                    #for i in range(4):
                        #if pastVictimInfo[i] == 0:
                            #pastVictimInfo[i] = victimInfo[i]
                    #victimInfo = pastVictimInfo
                #maz.victims[(self.x - 1, self.y, self.z)] = victimInfo
        elif nextTileInfo[1] > 4:
            if nextTileInfo[1] == 6:
                if (self.z % 10 == 0 and (self.x, self.y, self.z + 5) not in maz.maze) or (self.z % 10 != 0 and (self.x, self.y, self.z) not in maz.maze):
                    data = [0,0,0,0,2,1,0]
                    if robot.rampDir == -1:
                        data[(self.o + 2) % 4] = (int)(self.z / 10) + 4
                    data[(self.o + 1) % 4] = 1
                    data[(self.o + 3) % 4] = 1
                    maz.maze[(self.x, self.y, self.z + 5)] = data
                    maz.count += 1
                    maz.rampcount += 1
                    return -1
                if (self.x, self.y, self.z) not in maz.visited_ramps:
                    maz.count -= 1
                    maz.rampcount -= 1
                    maz.visited_ramps.append((self.x, self.y, self.z))
            if nextTileInfo[1] == 7:
                if (self.z % 10 == 0 and (self.x, self.y, self.z - 5) not in maz.maze) or (self.z % 10 != 0 and (self.x, self.y, self.z) not in maz.maze):
                    data = [0,0,0,0,2,1,0]
                    if robot.rampDir == -1:
                        data[(self.o + 2) % 4] = (int)(self.z / 10) + 4
                    data[(self.o + 1) % 4] = 1
                    data[(self.o + 3) % 4] = 1
                    maz.maze[(self.x, self.y, self.z - 5)] = data
                    maz.count += 1
                    maz.rampcount += 1
                    return -1
                if (self.x, self.y, self.z) not in maz.visited_ramps:
                    maz.count -= 1
                    maz.rampcount -= 1
                    maz.visited_ramps.append((self.x, self.y, self.z))
        else:
            if robot.rampDir == -1:
                if nextTileInfo[1] == 4:

                    maz.maze[(self.x, self.y, self.z)] = [1,1,1,1,1,0,0]

                    #if self.o == 0:
                        #maz.maze[(self.x, self.y + 1, self.z)] = [1,1,1,1,1,0,0]
                    #if self.o == 1:
                        #maz.maze[(self.x + 1, self.y, self.z)] = [1,1,1,1,1,0,0]
                    #if self.o == 2:
                        #maz.maze[(self.x, self.y - 1, self.z)] = [1,1,1,1,1,0,0]
                    #if self.o == 3:
                        #maz.maze[(self.x - 1, self.y, self.z)] = [1,1,1,1,1,0,0]
                    return -1
                else:

                    maz.coloured[(self.x, self.y, self.z)] = nextTileInfo[1]

                    #if self.o == 0:
                        #maz.coloured[(self.x, self.y + 1, self.z)] = nextTileInfo[1]
                    #if self.o == 1:
                        #maz.coloured[(self.x + 1, self.y, self.z)] = nextTileInfo[1]
                    #if self.o == 2:
                        #maz.coloured[(self.x, self.y - 1, self.z)] = nextTileInfo[1]
                    #if self.o == 3:
                        #maz.coloured[(self.x - 1, self.y, self.z)] = nextTileInfo[1]
            else:
                pass
        return 1
    def drop_cube(self):
        #print(maz.victims)
        if (self.x, self.y, self.z) in maz.victims:
            victimInfo = maz.victims[(self.x, self.y, self.z)]
            #print(victimInfo)
            for i in range(4):
                j = (i + self.o) % 4
                if victimInfo[j] == 10:
                    robot.drop_cube(i, 0)
                    victimInfo[j] = 20
                if victimInfo[j] == 11:
                    robot.drop_cube(i, 1)
                    victimInfo[j] = 21
                if victimInfo[j] == 12:
                    robot.drop_cube(i, 1)
                    victimInfo[j] = 22
                if victimInfo[j] == 13:
                    robot.drop_cube(i, 0)
                    victimInfo[j] = 23
                if victimInfo[j] == 14:
                    robot.drop_cube(i, 2)
                    victimInfo[j] = 24
                if victimInfo[j] == 15:
                    robot.drop_cube(i, 3)
                    victimInfo[j] = 25
            maz.victims[(self.x, self.y, self.z)] = victimInfo

    def nav(self):
        # clock.tick()
        print("---------- X: ", self.x, ", Y: ", self.y, ", Z: ", self.z, " ----------")
        print("---------- count: ", maz.count, ", rampcount: ", maz.rampcount, " ----------")
        info = self.Input()
        # if (self.x, self.y, self.z) in maz.maze:
        # newinfo = maze.save(info)
        # same = 1
        # for i in range(4):
        # if newinfo[i] != maze.maze[(self.x, self.y, self.z)][i]:
        # same = 0
        # break
        # if same == 0:
        # pass
        # print('robot input fps: ', clock.fps())
        # print(info)
        if info[5] == 1:
            #if self.z % 10 == 0:
            if (self.x, self.y, self.z - 5) in maz.maze or (self.x, self.y, self.z + 5) in maz.maze:
                #if robot.rampDir == 0:
                    #self.z -= 5
                #else:
                    #self.z += 5
                info[((self.o + 2) % 4 - self.o + 4) % 4] = (int)(self.prev / 10) + 4
            # info[6] == round(self.z - 0.5, ndigits=1)
            # info[7] == round(self.z + 0.5, ndigits=1)
        else:
            if self.ramp != -1:
                #temp = self.z
                #if self.ramp == 0:
                    #self.z -= 5
                #if self.ramp == 1:
                    #self.z += 5
                if (self.x, self.y, self.z) not in maz.maze:
                    self.prevT[self.o] = (int)(self.z / 10) + 4
                    if self.o == 0:
                        maz.maze[(self.x, self.y - 1, self.prev)] = self.prevT
                    if self.o == 1:
                        maz.maze[(self.x - 1, self.y, self.prev)] = self.prevT
                    if self.o == 2:
                        maz.maze[(self.x, self.y + 1, self.prev)] = self.prevT
                    if self.o == 3:
                        maz.maze[(self.x + 1, self.y, self.prev)] = self.prevT
        self.prevT = maz.save(info)
        print(info)

        #if robot.rampDir == -1:
            #self.ramp = -1
        #else:
            #if back == 0:
        self.ramp = robot.rampDir
            #if robot.rampDir == 0:
                #self.ramp = 1
            #else:
                #self.ramp = 0
        #if self.z % 10 == 0:
        self.prev = self.z
        # self.prevT = maz.maze[(self.x, self.y, self.z)]

        self.drop_cube()

        # robot.px = self.x
        # robot.py = self.y
        # robot.pz = self.z
        back = 0
        # info.pop(0)
        # print("ramp", robot.rampDir)
        if not self.path:
            if (self.x, self.y, self.z) not in maz.maze:
                if self.step == 0:
                    if info[2] != 1:
                        if info[5] == 0:
                            if maz.getCrds(2)[0] not in maz.maze and maz.getCrds(2)[1] not in maz.maze and \
                                    maz.getCrds(2)[2] not in maz.maze:
                                # print("from 2")
                                maz.count += 1
                                if info[5] == 1:
                                    maz.rampcount += 1
                            else:
                                maz.count -= 1
                                if info[5] == 1:
                                    maz.rampcount -= 1
                        else:
                            if maz.getCrds(2)[0] not in maz.maze and maz.getCrds(2)[1] not in maz.maze:
                                # print("from 2")
                                maz.count += 1
                                if info[5] == 1:
                                    maz.rampcount += 1
                            else:
                                maz.count -= 1
                                if info[5] == 1:
                                    maz.rampcount -= 1
                if info[3] != 1:
                    if info[5] == 0:
                        if maz.getCrds(3)[0] not in maz.maze and maz.getCrds(3)[1] not in maz.maze and maz.getCrds(3)[
                            2] not in maz.maze:
                            # print("from 3")
                            maz.count += 1
                            if info[5] == 1:
                                maz.rampcount += 1
                        else:
                            maz.count -= 1
                            if info[5] == 1:
                                maz.rampcount -= 1
                    else:
                        if maz.getCrds(3)[0] not in maz.maze and maz.getCrds(3)[1] not in maz.maze:
                            # print("from 3")
                            maz.count += 1
                            if info[5] == 1:
                                maz.rampcount += 1
                        else:
                            maz.count -= 1
                            if info[5] == 1:
                                maz.rampcount -= 1
                if info[0] != 1:
                    if info[5] == 0:
                        if maz.getCrds(0)[0] not in maz.maze and maz.getCrds(0)[1] not in maz.maze and maz.getCrds(0)[
                            2] not in maz.maze:
                            # print("from 0")
                            maz.count += 1
                            if info[5] == 1:
                                maz.rampcount += 1
                        else:
                            maz.count -= 1
                            if info[5] == 1:
                                maz.rampcount -= 1
                    else:
                        if maz.getCrds(0)[0] not in maz.maze and maz.getCrds(0)[1] not in maz.maze:
                            # print("from 0")
                            maz.count += 1
                            if info[5] == 1:
                                maz.rampcount += 1
                        else:
                            maz.count -= 1
                            if info[5] == 1:
                                maz.rampcount -= 1
                if info[1] != 1:
                    # print(info)
                    if info[5] == 0:
                        if maz.getCrds(1)[0] not in maz.maze and maz.getCrds(1)[1] not in maz.maze and maz.getCrds(1)[
                            2] not in maz.maze:
                            # print("from 1")
                            maz.count += 1
                            if info[5] == 1:
                                maz.rampcount += 1
                        else:
                            maz.count -= 1
                            if info[5] == 1:
                                maz.rampcount -= 1
                    else:
                        if maz.getCrds(1)[0] not in maz.maze and maz.getCrds(1)[1] not in maz.maze:
                            # print("from 1")
                            maz.count += 1
                            if info[5] == 1:
                                maz.rampcount += 1
                        else:
                            maz.count -= 1
                            if info[5] == 1:
                                maz.rampcount -= 1
            if info[5] == 1 and maz.count > maz.rampcount:
                maz.maze[(self.x, self.y, self.z)] = maz.save(info)
                maz.ramps.append((self.x, self.y, self.z))
                navi.optimalPath()
                back = 1
                if self.path:
                    print(self.path[0])
                    maz.printMap()
                    self.step += 1
                    self.go(maz.getDir(self.path.pop(0)))
            else:
                # if info[2] == 0:
                #     if self.getCrds(2) not in self.maze:
                #         self.count += 1
                #     else:
                #         self.count -= 1
                if (self.x, self.y, self.z) not in maz.maze:
                    if info[5] == 1:
                        maz.maze[(self.x, self.y, self.z)] = maz.save(info)
                        maz.ramps.append((self.x, self.y, self.z))
                    else:
                        maz.maze[(self.x, self.y, self.z)] = maz.save(info)
                maz.printMap()
                if info[0] != 1 and ((info[5] == 0 and maz.getCrds(0)[0] not in maz.maze and maz.getCrds(0)[
                    1] not in maz.maze and maz.getCrds(0)[2] not in maz.maze) or (
                                             info[5] == 1 and maz.getCrds(0)[0] not in maz.maze and maz.getCrds(0)[
                                         1] not in maz.maze)):
                    maz.count -= 1
                    if info[5] == 1:
                        maz.rampcount -= 1
                    self.step += 1
                    # self.pd = 2
                    self.go(0)
                elif info[3] != 1 and ((info[5] == 0 and maz.getCrds(3)[0] not in maz.maze and maz.getCrds(3)[
                    1] not in maz.maze and maz.getCrds(3)[2] not in maz.maze) or (
                                               info[5] == 1 and maz.getCrds(3)[0] not in maz.maze and maz.getCrds(3)[
                                           1] not in maz.maze)):
                    maz.count -= 1
                    if info[5] == 1:
                        maz.rampcount -= 1
                    self.step += 1
                    # self.pd = 1
                    self.go(3)
                elif info[1] != 1 and ((info[5] == 0 and maz.getCrds(1)[0] not in maz.maze and maz.getCrds(1)[
                    1] not in maz.maze and maz.getCrds(1)[2] not in maz.maze) or (
                                               info[5] == 1 and maz.getCrds(1)[0] not in maz.maze and maz.getCrds(1)[
                                           1] not in maz.maze)):
                    maz.count -= 1
                    if info[5] == 1:
                        maz.rampcount -= 1
                    self.step += 1
                    # self.pd = 3
                    self.go(1)
                elif info[2] != 1 and ((info[5] == 0 and maz.getCrds(2)[0] not in maz.maze and maz.getCrds(2)[
                    1] not in maz.maze and maz.getCrds(2)[2] not in maz.maze) or (
                                               info[5] == 1 and maz.getCrds(2)[0] not in maz.maze and maz.getCrds(2)[
                                           1] not in maz.maze)):
                    maz.count -= 1
                    if info[5] == 1:
                        maz.rampcount -= 1
                    self.step += 1
                    # self.pd = 0
                    self.go(2)
                else:
                    self.optimalPath()
                    if self.path:
                        print(self.path[0])
                        # maz.printMap()
                        self.step += 1
                        self.go(maz.getDir(self.path.pop(0)))
            # else:
            #     self.optimalPath()
            #     if self.path:
            #         self.step += 1
            #         robot.go(maz.getDir(self.path.pop(0)))
        else:
            print(self.path[0])
            maz.printMap()
            self.step += 1
            self.go(maz.getDir(self.path.pop(0)))
        #if robot.rampDir == -1:
            #self.ramp = -1
        #else:
            #if back == 0:
                #self.ramp = robot.rampDir
            #elif robot.rampDir == 0:
                #self.ramp = 1
            #else:
                #self.ramp = 0
        ##if self.z % 10 == 0:
        #self.prev = self.z
        ## self.prevT = maz.maze[(self.x, self.y, self.z)]
        if self.x == 0 and self.y == 0 and self.z == 0 and maz.count == 0:
            return 1
        return 0
    def optimalPath(self):
        print("finding optimal path")
        # turnRisk = 10
        # uTurnRisk = 15
        print(self.x, self.y, self.z)
        queue = []
        queue.append((self.x, self.y, self.z))
        cost = {}
        cost[(self.x, self.y, self.z)] = 0
        paths = {}
        paths[(self.x, self.y, self.z)] = []
        dest = []
        while queue:
            cur = queue.pop(0)
            prevo = self.o
            if len(paths[cur]) != 0:
                prevo = paths[cur][-1]
            for i in range(0, 4):
                if maz.maze[cur][i] != 1:
                    next = maz.neighbour(cur, i)
                    if next != -1:
                        turnv = 0
                        if prevo != -1:
                            if prevo == (i + 1) % 4 or prevo == (i + 3) % 4:
                                turnv = 0.6
                            if prevo == (i + 2) % 4:
                                turnv = 0.8
                        if next in cost:
                            if (cost[next] > cost[cur] + maz.maze[next][4] * (1 + turnv)):
                                cost[next] = cost[cur] + maz.maze[next][4] * (1 + turnv)
                                paths[next] = paths[cur].copy()
                                paths[next].append(i)
                                queue.append(next)
                        else:
                            cost[next] = cost[cur] + maz.maze[next][4] * (1 + turnv)
                            paths[next] = paths[cur].copy()
                            paths[next].append(i)
                            queue.append(next)
                    else:
                        dest.append(cur)
        time.sleep_ms(50)
        shortest = (0, 0, 0)
        minCost = 2000000000
        for i in dest:
            if shortest in maz.ramps and i not in maz.ramps:
                minCost = cost[i]
                shortest = i
            elif cost[i] < minCost:
                minCost = cost[i]
                shortest = i
        self.path = paths[shortest]
        print(minCost)
        for p in self.path:
            print(p, end=" ")
        print()
    def go(self, d):
        self.o = (self.o + d) % 4
        # if self.rampDir != -1:
        #     if self.rampDir == 0:
        #         navi.z -= 0.5
        #     else:
        #         navi.z += 0.5
        if self.o == 0:
            self.y += 1
        if self.o == 1:
            self.x += 1
        if self.o == 2:
            self.y -= 1
        if self.o == 3:
            self.x -= 1

        c = 0
        self.previous_r = robot.rampDir
        print("PREV RAMP ", self.previous_r)
        if self.z % 10 == 0:
            if (self.x, self.y, self.z + 5) in maz.maze:
                self.z += 5
                c = 5
                robot.rampDir = 1
            if (self.x, self.y, self.z - 5) in maz.maze:
                self.z -= 5
                c = -5
                robot.rampDir = 0
        else:
            if (self.x, self.y, self.z + 5) in maz.maze and robot.rampDir == 1:
                self.z += 5
                c = 5
            if (self.x, self.y, self.z - 5) in maz.maze and robot.rampDir == 0:
                self.z -= 5
                c = -5
            robot.rampDir = -1
        v = robot.go(d)
        if v == -1:
            if self.o == 0:
                self.y -= 1
            if self.o == 1:
                self.x -= 1
            if self.o == 2:
                self.y += 1
            if self.o == 3:
                self.x += 1
            self.z -= c
            robot.rampDir = self.previous_r
        print("RAMP DIR: ", robot.rampDir)
        # log.debug('go: ' + str(d))
    # pass
class Maze:
    def __init__(self):
        self.maze = {}
        self.coloured = {}
        self.victims = {}
        self.count = 0
        self.rampcount = 0
        self.ramps = []
        self.visited_ramps = []
        # self.path = []
    def neighbour(self, tile, d):
        if d == 0:
            if (self.maze[tile][5] != 1):
                if (tile[0], tile[1] + 1, tile[2]) in self.maze:
                    return (tile[0], tile[1] + 1, tile[2])
                if (tile[0], tile[1] + 1, tile[2] + 5) in self.maze:
                    return (tile[0], tile[1] + 1, tile[2] + 5)
                if (tile[0], tile[1] + 1, tile[2] - 5) in self.maze:
                    return (tile[0], tile[1] + 1, tile[2] - 5)
            else:
                if (tile[0], tile[1] + 1, tile[2]) in self.maze:
                    return (tile[0], tile[1] + 1, tile[2])
                elif (tile[0], tile[1] + 1, (self.maze[tile][d] - 4) * 10) in self.maze:
                    return (tile[0], tile[1] + 1, (self.maze[tile][d] - 4) * 10)
        if d == 1:
            if (self.maze[tile][5] != 1):
                if (tile[0] + 1, tile[1], tile[2]) in self.maze:
                    return (tile[0] + 1, tile[1], tile[2])
                if (tile[0] + 1, tile[1], tile[2] + 5) in self.maze:
                    return (tile[0] + 1, tile[1], tile[2] + 5)
                if (tile[0] + 1, tile[1], tile[2] - 5) in self.maze:
                    return (tile[0] + 1, tile[1], tile[2] - 5)
            else:
                if (tile[0] + 1, tile[1], tile[2]) in self.maze:
                    return (tile[0] + 1, tile[1], tile[2])
                elif (tile[0] + 1, tile[1], (self.maze[tile][d] - 4) * 10) in self.maze:
                    return (tile[0] + 1, tile[1], (self.maze[tile][d] - 4) * 10)
        if d == 2:
            if (self.maze[tile][5] != 1):
                if (tile[0], tile[1] - 1, tile[2]) in self.maze:
                    return (tile[0], tile[1] - 1, tile[2])
                if (tile[0], tile[1] - 1, tile[2] + 5) in self.maze:
                    return (tile[0], tile[1] - 1, tile[2] + 5)
                if (tile[0], tile[1] - 1, tile[2] - 5) in self.maze:
                    return (tile[0], tile[1] - 1, tile[2] - 5)
            else:
                if (tile[0], tile[1] - 1, tile[2]) in self.maze:
                    return (tile[0], tile[1] - 1, tile[2])
                elif (tile[0], tile[1] - 1, (self.maze[tile][d] - 4) * 10) in self.maze:
                    return (tile[0], tile[1] - 1, (self.maze[tile][d] - 4) * 10)
        if d == 3:
            if (self.maze[tile][5] != 1):
                if (tile[0] - 1, tile[1], tile[2]) in self.maze:
                    return (tile[0] - 1, tile[1], tile[2])
                if (tile[0] - 1, tile[1], tile[2] + 5) in self.maze:
                    return (tile[0] - 1, tile[1], tile[2] + 5)
                if (tile[0] - 1, tile[1], tile[2] - 5) in self.maze:
                    return (tile[0] - 1, tile[1], tile[2] - 5)
            else:
                if (tile[0] - 1, tile[1], tile[2]) in self.maze:
                    return (tile[0] - 1, tile[1], tile[2])
                elif (tile[0] - 1, tile[1], (self.maze[tile][d] - 4) * 10) in self.maze:
                    return (tile[0] - 1, tile[1], (self.maze[tile][d] - 4) * 10)
        return -1
    def getCrds(self, d):
        n = (navi.o + d) % 4
        if n == 0:
            if robot.rampDir == -1:
                return (navi.x, navi.y + 1, navi.z), (navi.x, navi.y + 1, navi.z + 5), (navi.x, navi.y + 1, navi.z - 5)
            elif robot.rampDir == 1:
                return (navi.x, navi.y + 1, navi.z), (navi.x, navi.y + 1, navi.z + 5)
            else:
                return (navi.x, navi.y + 1, navi.z), (navi.x, navi.y + 1, navi.z - 5)
        if n == 1:
            if robot.rampDir == -1:
                return (navi.x + 1, navi.y, navi.z), (navi.x + 1, navi.y, navi.z + 5), (navi.x + 1, navi.y, navi.z - 5)
            elif robot.rampDir == 1:
                return (navi.x + 1, navi.y, navi.z), (navi.x + 1, navi.y, navi.z + 5)
            else:
                return (navi.x + 1, navi.y, navi.z), (navi.x + 1, navi.y, navi.z - 5)
        if n == 2:
            if robot.rampDir == -1:
                return (navi.x, navi.y - 1, navi.z), (navi.x, navi.y - 1, navi.z + 5), (navi.x, navi.y - 1, navi.z - 5)
            elif robot.rampDir == 1:
                return (navi.x, navi.y - 1, navi.z), (navi.x, navi.y - 1, navi.z + 5)
            else:
                return (navi.x, navi.y - 1, navi.z), (navi.x, navi.y - 1, navi.z - 5)
        if n == 3:
            if robot.rampDir == -1:
                return (navi.x - 1, navi.y, navi.z), (navi.x - 1, navi.y, navi.z + 5), (navi.x - 1, navi.y, navi.z - 5)
            elif robot.rampDir == 1:
                return (navi.x - 1, navi.y, navi.z), (navi.x - 1, navi.y, navi.z + 5)
            else:
                return (navi.x - 1, navi.y, navi.z), (navi.x - 1, navi.y, navi.z - 5)
    def getDir(self, d):
        return (d - navi.o + 4) % 4
    def printMap(self):
        pass
        # print(navi.x, navi.y, navi.z, navi.o)
        # print(robot.X, robot.Y, robot.Z)
        # print(self.count, self.rampcount, navi.step)
        # for k in range (-SZ, H-SZ):
        # for i in range(SY, SY-L, -1):
        # for j in range(-SX, W-SX):
        # if j == navi.x and i == navi.y and (k * 10 == navi.z or k * 10 + 5 == navi.z or k * 10 - 5 == navi.z):
        # print('$robot$', end = ", ")
        # else:
        # if (j, i, k * 10) in self.maze:
        # print(''.join(str (e) for e in self.maze[(j, i, k * 10)]), end = ", ")
        # elif (j, i, k * 10 + 5) in self.maze:
        # print(''.join(str (e) for e in self.maze[(j, i, k * 10 + 5)]), end = ", ")
        # elif (j, i, k * 10 - 5) in self.maze:
        # print(''.join(str (e) for e in self.maze[(j, i, k * 10 - 5)]), end = ", ")
        # else:
        # print('*******', end = ", ")
        # print()
        # print()
        # print()
    def save(self, info):
        out = info.copy()
        if navi.o == 1:
            t = out[3]
            out[3] = out[2]
            out[2] = out[1]
            out[1] = out[0]
            out[0] = t
        if navi.o == 2:
            t = out[0]
            out[0] = out[2]
            out[2] = t
            t = out[1]
            out[1] = out[3]
            out[3] = t
        if navi.o == 3:
            t = out[1]
            out[1] = out[2]
            out[2] = out[3]
            out[3] = out[0]
            out[0] = t
        return out
    def update(self, info, x, y, z):
        # if z % 10 == 0:
        pass
    def rampLev(self, l, d):
        if d == 0:
            return l - 0.5
        else:
            return l + 0.5
    def black(self, a, b):
        self.maze[(a, b)] = [1, 1, 1, 1, 1]
        pass
def flash_blue(count=10):
    led = LED(3)
    for i in range(count):
        led.on()
        time.sleep_ms(500)
        led.off()
        time.sleep_ms(500)
def flash_red(count=5):
    led = LED(1)
    for i in range(count):
        led.on()
        time.sleep_ms(500)
        led.off()
        time.sleep_ms(500)
def flash_green(count=5):
    led = LED(2)
    for i in range(count):
        led.on()
        time.sleep_ms(500)
        led.off()
        time.sleep_ms(500)
def flash(count = 5):
    for i in range(count):
        p1.value(1)
        time.sleep_ms(500)
        p1.value(0)
        time.sleep_ms(500)
#SystemExit
while True:
    print("\n\n\nstart...")
    p1.value(1)
    #flash_blue(3)
    time.sleep_ms(3000)
    robot = Robot()
    navi = Nav()
    maz = Maze()
    #print(list(range(360)))
    #print(list(robot.lidar_data.data))

    while True:
        robot.cam.img = sensor.snapshot()
        v = robot.motor.voltage()
        print('voltage: ', v)
        if v > 7: break
        # flash_red(3)
        p1.value(1)
        time.sleep_ms(100)
        p1.value(0)
        time.sleep_ms(100)
    #flash_green(3)
    for i in range(3):
        p1.value(1)
        time.sleep_ms(500)
        p1.value(0)
        time.sleep_ms(500)
    robot.get_lidardata()
    #for i in range(360):
        #print(i, end = ",")
    #print()
    for i in range(360):
        print(i, ":", robot.lidar_data.data[i], end = ",  ")
    print()
    # m = Maze()
    try:
        # robot.fixPos()
        # robot.go(0)
        # TODO get minial distance to walls as reference for future
        for i in range(5):
            robot.check_wall_reference()
            print('wall ref:', robot.wall_ref)
        # ds=robot.lidar_data.get_distance()
        # print(ds)
        # print(robot.lidar_data.data)
        # print(min(robot.lidar_data.data))
        # while True:
        # time.sleep_ms(3000)
        # robot.adjust_to_wall()

        #while True:
            #robot.adjust_left_and_right()
            #time.sleep_ms(5000)
        restart = False
        # robot.forward_tile()
        while True:
            # robot.forward();
            if navi.nav() == 1: break
            time.sleep_ms(10)
            if button_pressed == 1:
                robot.motor.stop()
                proc_button()
                restart = True
                break
            # robot.get_lidardata()
            # d = robot.ldata
            # print(d[0],d[180],d[0]-d[180],d[90],d[270])
            # time.sleep_ms(200)
        # print('tick')
        # robot.adjust_angle()
        if restart: continue
        flash_blue()
    except KeyboardInterrupt:
        print("Ctrl-C")
    finally:
        # log.flush()
        # robot.motor2.stop()
        robot.motor.stop()
    print("finished")
    # robot.Input()
    print(navi.x, navi.y, navi.y, navi.step)
    maz.printMap()
