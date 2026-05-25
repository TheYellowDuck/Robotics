from pyb import millis, LED
import pyb
from machine import I2C, Pin
from pyb2 import *
import sensor, image, time, math
import sys
import gc

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

red_threshold = (0, 100, 15, 37, 6, 25)  # L A B
green_threshold = (0, 100, -27, -10, 1, 23)  # L A B
yellow_threshold = (0, 100, -15, 1, 21, 63)  # L A B
blue_threshold = (0, 100, -1, 13, -25, -8)  # L A B

black_threshold = (17, 38, -7, 5, -8, 9)
white_threshold = (75, 90, -4, 9, 1, 16)
silver_threshold = (64, 73, -6, 6, -8, 8)

letter_threshold = (36, 58, -4, 10, -8, 4)


w = 320
h = 240
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
        r = self.motor.is_stopped()
        return r == 0
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
        self.stop()
        s = p2 + 4096 * c
        return s
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
    def drop_cube(self):
        servo9.set(ms=1-0.5)
        time.sleep_ms(400)
        servo9.set(ms=2+0.5)
        time.sleep_ms(250)
        servo9.set(ms=1.5)
        time.sleep_ms(500)
        print("Dropped Cube")
        flash(1)
    def check_tile(self):
        #1: stair
        #2: ramp
        cramp = robot.check_ramp()
        if cramp == 0:
            return 1
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

    def scan(self):
        self.img = sensor.snapshot()
        self.img.gaussian(1)
        self.img.draw_rectangle((int)(w/8), (int)(h*3/5 + 10), (int)(w*6/8), h - ((int)(h*3/5)), color = (0, 0, 0),fill = True)

    def check_victims(self, blobs1, blobs2):
        #10: G
        #11: Y
        #12: R
        #13: U
        #14: S
        #15: H
        arr = [0,0,0,0]

        temp = [self.get_color_wall(blobs1), self.get_letter(blobs2)]
        for i in range(4):
            for j in range(2):
                if temp[j][i] != 0:
                    arr[i] = temp[j][i]
                    break
        return arr

    def find_blob_wall(self, c):
        self.scan()

        blobs = []
        blobs2 = []
        blobs3 = []

        for i in range(c):
            blobs2.append([])
            for blob in self.img.find_blobs([red_threshold, green_threshold, yellow_threshold], pixels_threshold=100, area_threshold=180, merge=False, roi = (0, (int)(h/10), w, h - (int)(h/10))):
                if blob.area() <= 1000 and blob.h() >= blob.w() - 5 and blob.density() >= 0.5 and blob.elongation() < 0.9:
                    blobs2[i].append(blob)
        for i in blobs2:
            for j in i:
                if len(blobs3) == 0:
                    blobs3.append(j)
                for k in blobs3:
                    if j.code() == k.code() and abs(j.cx() - k.cx()) < 10 and abs(j.cy() - k.cy()) < 10:
                        blobs.append(k)
                        blobs3.remove(k)
                    else:
                        blobs3.append(j)
        return blobs

    def get_color_wall(self, blobs):
        fov = 100

        arr = [0,0,0,0]

        for blob in blobs:
            self.img.draw_rectangle(blob.rect())

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

            if -20 < blob_angle and blob_angle < 20 and robot.ck_walls[1][0][1] != 0:
                if blob.code() == 1:
                    arr[0] = 12
                if blob.code() == 2:
                    arr[0] = 10
                if blob.code() == 4:
                    arr[0] = 11
            elif blob_angle <= -20 and robot.ck_walls[1][0][0] != 0 and blob.elongation() > 0.5:
                if blob.code() == 1:
                    arr[3] = 12
                if blob.code() == 2:
                    arr[3] = 10
                if blob.code() == 4:
                    arr[3] = 11
            elif robot.ck_walls[1][0][2] != 0 and blob.elongation() > 0.5:
                if blob.code() == 1:
                    arr[1] = 12
                if blob.code() == 2:
                    arr[1] = 10
                if blob.code() == 4:
                    arr[1] = 11

        return arr

    def find_blob_ground(self, c):
        self.scan()

        blobs = []
        blobs2 = []
        blobs3 = []

        for i in range(c):
            blobs2.append([])
            for blob in self.img.find_blobs([white_threshold, silver_threshold, blue_threshold, black_threshold], pixels_threshold=1500, area_threshold=3500, merge=False, roi = ((int)(w/8), (int)(h/3) + 10, (int)(w*6/8), (int)(h/3) - 20)):
                blobs2[i].append(blob)
        for i in blobs2:
            for j in i:
                if len(blobs3) == 0:
                    blobs3.append(j)
                for k in blobs3:
                    if j.code() == k.code() and abs(j.cx() - k.cx()) < 30 and abs(j.cy() - k.cy()) < 30:
                        blobs.append(k)
                        blobs3.remove(k)
                    else:
                        blobs3.append(j)

        return blobs


    def get_color_ground(self, blobs):

        #1: white
        #2: silver
        #3: blue
        #4: black

        for blob in blobs:

            self.img.draw_rectangle(blob.rect())

            if blob.code() == 1:
                self.img.draw_string(blob.x() + 2, blob.y() + 2, "white")
                return 1
            if blob.code() == 2:
                self.img.draw_string(blob.x() + 2, blob.y() + 2, "silver")
                return 2
            if blob.code() == 4:
                self.img.draw_string(blob.x() + 2, blob.y() + 2, "blue")
                return 3
            if blob.code() == 8:
                self.img.draw_string(blob.x() + 2, blob.y() + 2, "black")
                return 4

        return 3

    def check_threshold(self, pixel):
        for i in range(6):
            if i % 2 == 0 and letter_threshold[i] > pixel[(int) (i / 2)]:
                return 0
            if i % 2 == 1 and letter_threshold[i] < pixel[(int) (i / 2)]:
                return 0
        return 1

    def check_letter(self, blob):

        if (blob.x() == 0 or blob.x() + blob.w() == w):
            return -1
        pixel = self.img.get_pixel(blob.cx() + 2, blob.cy() + 2)
        pixel = image.rgb_to_lab(pixel)
        pixel2 = self.img.get_pixel(blob.cx() - 2, blob.cy() - 2)
        pixel2 = image.rgb_to_lab(pixel2)
        if self.check_threshold(pixel) == 1 or self.check_threshold(pixel2) == 1:
            return -1

        print("Text Blob", blob.pixels(), blob.area())
        pixelcount = 0
        for i in range(blob.cy() - 5, blob.cy() + 10):
            pixel = self.img.get_pixel(blob.cx(), i)
            pixel = image.rgb_to_lab(pixel)
            if self.check_threshold(pixel) == 1:
                pixelcount += 1
        if pixelcount < 2:
            self.img.draw_string(blob.x() + 2, blob.y() + 2, "U")
            return 13

        pixelcount = 0
        pixelcount2 = 0
        pixelcount3 = 0
        count = 0

        for i in range(blob.y(), blob.y() + blob.h()):
            pixel = self.img.get_pixel(blob.cx(), i)
            pixel = image.rgb_to_lab(pixel)
            if count == 0 and self.check_threshold(pixel) == 1:
                pixelcount += 1
                count = 1
            else:
                count = 0
        count = 0
        count2 = 0
        for i in range(blob.x(), blob.x() + blob.w()):
            pixel = self.img.get_pixel(i, blob.cy() - 3)
            pixel = image.rgb_to_lab(pixel)
            if count == 0 and self.check_threshold(pixel) == 1:
                pixelcount2 += 1
                count = 1
            else:
                count = 0
            pixel = self.img.get_pixel(i, blob.cy() + 3)
            pixel = image.rgb_to_lab(pixel)
            if count2 == 0 and self.check_threshold(pixel) == 1:
                pixelcount3 += 1
                count2 = 1
            else:
                count2 = 0
        print(pixelcount, pixelcount2, pixelcount3)
        if pixelcount <= 2 and pixelcount2 <= 2 and pixelcount3 <= 2:
            self.img.draw_string(blob.x() + 2, blob.y() + 2, "H")
            return 15
        self.img.draw_string(blob.x() + 2, blob.y() + 2, "S")
        return 14

    def find_blob_letter(self, c):
        self.scan()

        blobs = []
        blobs2 = []
        blobs3 = []

        for i in range(c):
            blobs2.append([])
            for blob in self.img.find_blobs([letter_threshold], pixels_threshold=30, area_threshold=200, merge=True, roi = (0, (int)(h/10), w, h - (int)(h/10))):
                if blob.area() <= 1000 and ((blob.h() >= blob.w() - 5 and blob.h() < blob.w() + 15) or (blob.w() >= blob.h() - 5 and blob.w() < blob.h() + 15)) and blob.density() < 0.65 and blob.elongation() < 0.9:
                    blobs2[i].append(blob)
        for i in blobs2:
            for j in i:
                if len(blobs3) == 0:
                    blobs3.append(j)
                for k in blobs3:
                    if j.code() == k.code() and abs(j.cx() - k.cx()) < 10 and abs(j.cy() - k.cy()) < 10:
                        blobs.append(k)
                        blobs3.remove(k)
                    else:
                        blobs3.append(j)
        return blobs

    def get_letter(self, blobs):
        fov = 100

        arr = [0,0,0,0]

        for blob in blobs:
            self.img.draw_rectangle((blob.x() - 5, blob.y() - 5, blob.w() + 10, blob.h() + 10))
            bound = blob.rect()
            cx = blob.cx()
            cy = blob.cy()
            print(bound, cx, cy)

            blob_angle_to_robot = (cx - w/2.0)*fov/w

            blob_angle = robot.ld_ang + blob_angle_to_robot

            print(robot.ld_ang, blob_angle_to_robot, blob_angle)

            if -20 < blob_angle and blob_angle < 20:
                if arr[0] == 0 and robot.ck_walls[1][0][1] != 0:
                    v = self.check_letter(blob)
                    if v != -1:
                        arr[0] = v
            elif blob_angle <= -20 and robot.ck_walls[1][0][0] != 0 and blob.elongation() > 0.5:
                if arr[3] == 0:
                    v = self.check_letter(blob)
                    if v != -1:
                        arr[3] = v
            elif robot.ck_walls[1][0][2] != 0 and blob.elongation() > 0.5:
                if arr[1] == 0:
                    v = self.check_letter(blob)
                    if v != -1:
                        arr[1] = v

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
        self.adjustCount = 0
    def check_walls(self):
        self.get_lidardata()
        return self.ck_walls[0]
    def check_tile_info(self):
        self.get_lidardata()

        rtile = 1
        rvictims = [0]*4

        tiletype = 1
        if (navi.x, navi.y, navi.z) in maz.maze:
            tiletype = maz.maze[(navi.x, navi.y, navi.z)][5]
        if (navi.x, navi.y, navi.z) in maz.coloured:
            tiletype = maz.coloured[(navi.x, navi.y, navi.z)]
        elif (navi.o == 0 and (navi.x, navi.y - 1, navi.z) in maz.maze and maz.maze[(navi.x, navi.y - 1, navi.z)][5] <= 4) or \
        (navi.o == 1 and (navi.x, navi.y - 1, navi.z) in maz.maze and maz.maze[(navi.x - 1, navi.y, navi.z)][5] <= 4) or \
        (navi.o == 2 and (navi.x, navi.y - 1, navi.z) in maz.maze and maz.maze[(navi.x, navi.y + 1, navi.z)][5] <= 4) or \
        (navi.o == 3 and (navi.x, navi.y - 1, navi.z) in maz.maze and maz.maze[(navi.x + 1, navi.y, navi.z)][5] <= 4):
            tiletype = self.motor.check_tile() + 4
        tile = [1] * 3
        victims = [[0]*4]*3

        if tiletype == 4:
            print("ground")
            blobs = self.cam.find_blob_ground(3)
            for i in range(3):
                tile[i] = self.cam.get_color_ground(blobs)
            rtile = tile[0]
            if rtile != tile[1] and rtile != tile[2] and tile[1] == tile[2]:
                rtile = tile[1]
        elif tiletype == 7:
            print("ground")
            blobs = self.cam.find_blob_ground(3)
            for i in range(3):
                tile[i] = self.cam.get_color_ground(blobs)
            isramp = tile[0]
            if isramp != tile[1] and isramp != tile[2] and tile[1] == tile[2]:
                isramp = tile[1]
            if isramp != 1:
                rtile = isramp
            else:
                rtile = 7
        else:
            rtile = tiletype

        if rtile == 1:
            print("wall")
            color = self.cam.find_blob_wall(3)
            letter = self.cam.find_blob_letter(3)

            for i in range(3):
                victims[i] = self.cam.check_victims(color, letter)
        rvictims = victims[0]
        for i in range(4):
            if rvictims[i] != victims[1][i] and rvictims[i] != victims[2][i] and victims[1][i] == victims[2][i]:
                rvictims[i] = victims[1][i]

        print("Image Recognition Done")
        return [rvictims, rtile]
    def check_front(self):
        front = sensor1.read()
        print('front ', sensor1.read())
        if (((navi.x, navi.y, navi.z) in maz.maze and maz.maze[(navi.x, navi.y, navi.z)][5] <= 4) and \
        ((navi.o == 0 and maz.maze[(navi.x, navi.y - 1, navi.z)][5] <= 4) or \
        (navi.o == 1 and maz.maze[(navi.x - 1, navi.y, navi.z)][5] <= 4) or \
        (navi.o == 2 and maz.maze[(navi.x, navi.y + 1, navi.z)][5] <= 4) or \
        (navi.o == 3 and maz.maze[(navi.x + 1, navi.y, navi.z)][5] <= 4))) and \
        (front < 130 or (self.prev_dist < 1000 and self.prev_dist - front > 330)):
            return True
        return False
    def adjust(self):
        self.adjust_angle()
        self.adjust_fb()
        self.adjust_left_and_right()
    def forward(self):
        v = navi.tileInput()
        if v == -1:
            return -1

        self.get_lidardata()
        if robot.rampDir == 1:
            startT = millis()
            self.motor.run(2500, 2500)
            time.sleep_ms(3150)
            front = sensor1.read()
            while front > 1500:
                front = sensor1.read()
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
        else:
            self.prev_dist = self.lidar_data.data[0]
            self.motor.forward_ms(2500, 2500, self.check_front)
        if ((navi.x, navi.y, navi.z) in maz.maze and maz.maze[(navi.x, navi.y, navi.z)][5] >= 5) or \
        ((navi.x, navi.y, navi.z) in maz.coloured and maz.coloured[(navi.x, navi.y, navi.z)] >= 5):
            return 1
        self.adjust_angle()
        self.adjust_fb()
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
    def adjust_left_and_right(self):
        self.get_lidardata()
        d1 = self.lidar_data.get_distance([90])[0] - 5
        if d1 >= 300 and self.ld_edges[1] - 5 < 300:
            d1 = self.ld_edges[1] - 5
        d2 = self.lidar_data.get_distance([270])[0] + 5
        if d2 >= 300 and self.ld_edges[3] + 5 < 300:
            d2 = self.ld_edges[3] + 5

        front = self.lidar_data.data[0]
        if front >= 200 and self.ld_edges[0] < 200:
            front = self.ld_edges[0]

        print(d1, d2)
        if d1 < 100 or (d2 < 300 and d2 > 200):
            if front < 200:
                if d1 < 100:
                    self.shift_pos(-1000, (int) ((150 - d1) * 7.5), 2)
                else:
                    self.shift_pos(-1000, (int) ((d2 - 150) * 7.5), 2)
            else:
                if d1 < 100:
                    self.shift_pos(-1000, (int) ((150 - d1) * 10), 1)
                else:
                    self.shift_pos(-1000, (int) ((d2 - 150) * 10), 1)
        elif d2 < 100 or (d1 < 300 and d1 > 200):
            if front < 200:
                if d2 < 100:
                    self.shift_pos(1000, (int) ((150 - d2) * 7.5), 2)
                else:
                    self.shift_pos(1000, (int) ((d1 - 150) * 7.5), 2)
            else:
                if d2 < 100:
                    self.shift_pos(1000, (int) ((150 - d2) * 10), 1)
                else:
                    self.shift_pos(1000, (int) ((d1 - 150) * 10), 1)
    def adjust_fb(self):
        self.get_lidardata()
        if self.lidar_data.get_distance([0])[0] < 250 and sensor1.read() > self.lidar_data.get_distance([0])[0] - 50:
            self.adjust_to_front()
        elif self.lidar_data.get_distance([180])[0] < 250:
            self.adjust_to_back()
    def adjust_to_front(self):
        d = self.lidar_data.get_distance([0])[0]
        if d >= 250:
            d = self.ld_edges[0]
        print("adjust front")
        if d < 90:
            self.motor.run(-2000, -2000)
            time.sleep_ms((int)((120 - d) * 8))
            self.motor.stop()
            time.sleep_ms(200)
        if d < 250 and d > 130:
            self.motor.run(2000, 2000)
            time.sleep_ms((int)((d - 110) * 10))
            self.motor.stop()
            time.sleep_ms(200)
    def adjust_to_back(self):
        d = self.lidar_data.get_distance([180])[0]
        if d >= 250:
            d = self.ld_edges[2]
        print("adjust back")
        if d < 90:
            self.motor.run(2000, 2000)
            time.sleep_ms((int)((120 - d) * 8))
            self.motor.stop()
            time.sleep_ms(200)
        if d < 250 and d > 130:
            self.motor.run(-2000, -2000)
            time.sleep_ms((int)((d - 110) * 10))
            self.motor.stop()
            time.sleep_ms(200)
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
            self.adjustCount += 1
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
        elif abs(self.ld_ang) <= 27.5:
            time.sleep_ms(750)
        elif abs(self.ld_ang) <= 30:
            time.sleep_ms(775)
        elif abs(self.ld_ang) <= 32.5:
            time.sleep_ms(800)
        elif abs(self.ld_ang) <= 35:
            time.sleep_ms(825)
        elif abs(self.ld_ang) <= 37.5:
            time.sleep_ms(850)
        elif abs(self.ld_ang) <= 40:
            time.sleep_ms(875)
        elif abs(self.ld_ang) <= 42.5:
            time.sleep_ms(900)
        else:
            self.motor.stop()
            flash_red(5)
        self.motor.stop()
        time.sleep_ms(200)
    def right(self):
        self.adjust_left_and_right()
        self.motor.turn_right()
        self.adjustCount = 0
        self.adjust_angle()
        self.adjust_angle()
        if self.adjustCount <= 1:
            self.adjust_left_and_right()
        return self.forward()
    def back(self):
        self.adjust_left_and_right()
        self.motor.turn_back()
        self.adjust_angle()
        self.adjust_angle()
        return self.forward()
    def left(self):
        self.adjust_left_and_right()
        self.motor.turn_left()
        self.adjustCount = 0
        self.adjust_angle()
        self.adjust_angle()
        if self.adjustCount <= 1:
            self.adjust_left_and_right()
        return self.forward()
    def get_lidardata(self):
        self.ldata = self.lidar.read(timeout=3000, percent=100)
        self.ldata = np.array(self.ldata, dtype=np.float)
        self.lidar_data.data = self.ldata
        self.ck_walls = self.lidar_data.check_walls()
        self.ld_lines = self.lidar_data.find_lines()
        ldedges = self.lidar_data.find_edge()
        self.ld_edges = [10000] * 4
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
                cnt += 1
                sum += mm
        if cnt > 0:
            self.ld_ang = sum / cnt
        else:
            self.ld_ang = 0
        edge_ang = [-1] * 4
        for i in ldedges:
            if self.ldata[i] > 100 and self.ldata[i] < 300:
                if i >= self.ld_ang and i < self.ld_ang + 90:
                    edge_ang[0] = i
                if i >= self.ld_ang + 90 and i < self.ld_ang + 180:
                    edge_ang[1] = i
                if i >= self.ld_ang + 180 and i < self.ld_ang + 270:
                    edge_ang[2] = i
                if (i >= self.ld_ang + 270 and i < 360) or (i < self.ld_ang):
                    edge_ang[3] = i
        for i in range(4):
            if edge_ang[i] != -1:
                v = math.sin(((edge_ang[i] + self.ld_ang + 90) % 90) * math.pi / 180) * self.ldata[edge_ang[i]]
                if v < 300:
                    self.ld_edges[i] = v
            if edge_ang[(i + 3) % 4] != -1:
                v = math.cos(((edge_ang[(i + 3) % 4] + self.ld_ang + 90) % 90) * math.pi / 180) * self.ldata[edge_ang[(i + 3) % 4]]
                if v < 300:
                    if self.ld_edges[i] == 10000:
                        self.ld_edges[i] = v
                    else:
                        self.ld_edges[i] += v
                        self.ld_edges[i] /= 2
        print(self.ld_edges[i])
    def check_ramp(self):
        self.get_lidardata()
        front = self.lidar_data.data[0]
        front2 = sensor1.read()
        down = sensor2.read()
        print("front dist:", front, front2, navi.previous_r)
        if front > 500 and front2 < 380 and down < 500:
            print("stair")
            return 0
        #if front <= 500 and front - 70 > front2 and front - 230 < front2:
            #print("ramp")
            #maz.ramps.append((navi.x, navi.y, navi.z))
            #return 1
        #if navi.previous_r == -1 and down > 500:
            #print("ramp")
            #maz.ramps.append((navi.x, navi.y, navi.z))
            #return 2
        return
    def drop_cube(self, d, count):
        print("DROPPING")
        if d == 3:
            flash()
            for i in range(count):
                self.motor.drop_cube()
        if d == 0:
            self.motor.turn_right()
            flash()
            for i in range(count):
                self.motor.drop_cube()
            self.motor.turn_left()
        if d == 1:
            self.motor.turn_back()
            flash()
            for i in range(count):
                self.motor.drop_cube()
            self.motor.turn_back()
        if d == 2:
            self.motor.turn_left()
            flash()
            time.sleep_ms(1500)
            for i in range(count):
                self.motor.drop_cube()
            self.motor.turn_right()
        print("Finished Dropping")
    def go(self, d):
        if (d == 0):  # go forwards
            return self.forward()
        if (d == 1):  # go right
            return self.right()
        if (d == 2):  # go back
            return self.back()
        if (d == 3):  # go left
            return self.left()
class Nav:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.o = 0
        self.step = 0
        self.previous_r = -1
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
            return input
        input = robot.check_walls()
        tcolor = 1
        if (self.x, self.y, self.z) in maz.coloured:
            tcolor = maz.coloured[(self.x, self.y, self.z)]
        if tcolor == 0:
            input.append(1)
        if tcolor == 1:
            input.append(1)
        if tcolor == 2:
            input.append(1)
        if tcolor == 3:
            input.append(3)
        if tcolor == 4:
            input.append(999)
        if tcolor == 5:
            input.append(3)
        if tcolor == 6:
            input.append(4)
        if tcolor == 7:
            input.append(4)
        if len(input) == 5:
            input.append(0)
        if (self.x, self.y, self.z) in maz.coloured:
            input[5] = tcolor
        if len(input) == 6:
            input.append(0)
        return input

    def tileInput(self):
        nextTileInfo = robot.check_tile_info()
        robot.get_lidardata()
        print("----- ", nextTileInfo, " -----")
        if nextTileInfo[1] <= 3:
            if robot.rampDir == -1 and nextTileInfo[1] != 1:
                maz.coloured[(self.x, self.y, self.z)] = nextTileInfo[1]
            victimInfo = maz.save(nextTileInfo[0])
            print(victimInfo)

            if (self.x, self.y, self.z) in maz.victims:
                pastVictimInfo = maz.victims[(self.x, self.y, self.z)]
                for i in range(4):
                    if pastVictimInfo[i] == 0:
                        pastVictimInfo[i] = victimInfo[i]
                victimInfo = pastVictimInfo
            maz.victims[(self.x, self.y, self.z)] = victimInfo

            noNeed = True
            data = [0,0,0,0,1,0,0]
            if (self.o == 0 and maz.maze[(self.x, self.y - 1, self.z)][5] >= 5) or \
                (self.o == 1 and maz.maze[(self.x - 1, self.y, self.z)][5] >= 5) or \
                (self.o == 2 and maz.maze[(self.x, self.y + 1, self.z)][5] >= 5) or \
                (self.o == 3 and maz.maze[(self.x + 1, self.y, self.z)][5] >= 5):
                noNeed = False
                data[self.o] = 1
            elif victimInfo[0] != 0 or ((robot.ck_walls[1][0][1] != 1 or sensor1.read() < robot.lidar_data.get_distance([0])[0] - 70) and (
                (self.o == 0 and (self.x, self.y + 1, self.z) not in maz.maze) or \
                (self.o == 1 and (self.x + 1, self.y, self.z) not in maz.maze) or \
                (self.o == 2 and (self.x, self.y - 1, self.z) not in maz.maze) or \
                (self.o == 3 and (self.x - 1, self.y, self.z) not in maz.maze))):
                noNeed = False
                data[self.o] = 1
            elif victimInfo[1] != 0 or (robot.ck_walls[1][0][2] != 1 and (
                (self.o == 3 and ((self.x, self.y + 1, self.z) not in maz.maze and (self.x, self.y + 1, self.z + 5) not in maz.maze and (self.x, self.y + 1, self.z - 5) not in maz.maze)) or \
                (self.o == 0 and ((self.x + 1, self.y, self.z) not in maz.maze and (self.x + 1, self.y, self.z + 5) not in maz.maze and (self.x + 1, self.y, self.z - 5) not in maz.maze)) or \
                (self.o == 1 and ((self.x, self.y - 1, self.z) not in maz.maze and (self.x, self.y - 1, self.z + 5) not in maz.maze and (self.x, self.y - 1, self.z - 5) not in maz.maze)) or \
                (self.o == 2 and ((self.x - 1, self.y, self.z) not in maz.maze and (self.x - 1, self.y, self.z + 5) not in maz.maze and (self.x - 1, self.y, self.z - 5) not in maz.maze)))):
                noNeed = False
                data[(self.o + 1)%4] = 1
            elif victimInfo[3] != 0 or (robot.ck_walls[1][0][0] != 1 and (
                (self.o == 1 and ((self.x, self.y + 1, self.z) not in maz.maze and (self.x, self.y + 1, self.z + 5) not in maz.maze and (self.x, self.y + 1, self.z - 5) not in maz.maze)) or \
                (self.o == 2 and ((self.x + 1, self.y, self.z) not in maz.maze and (self.x + 1, self.y, self.z + 5) not in maz.maze and (self.x + 1, self.y, self.z - 5) not in maz.maze)) or \
                (self.o == 3 and ((self.x, self.y - 1, self.z) not in maz.maze and (self.x, self.y - 1, self.z + 5) not in maz.maze and (self.x, self.y - 1, self.z - 5) not in maz.maze)) or \
                (self.o == 0 and ((self.x - 1, self.y, self.z) not in maz.maze and (self.x - 1, self.y, self.z + 5) not in maz.maze and (self.x - 1, self.y, self.z - 5) not in maz.maze)))):
                noNeed = False
                data[(self.o + 3)%4] = 1

            if noNeed:
                print("not going")
                maz.maze[(self.x, self.y, self.z)] = data
                return -1
        elif nextTileInfo[1] > 4:
            if nextTileInfo[1] == 5:
                maz.coloured[(self.x, self.y, self.z)] = nextTileInfo[1]
            elif nextTileInfo[1] == 6:
                if (self.z % 10 == 0 and (self.x, self.y, self.z + 5) not in maz.maze) or (self.z % 10 != 0 and (self.x, self.y, self.z) not in maz.maze):
                    data = [0,0,0,0,2,6,0]
                    if robot.rampDir == -1:
                        data[(self.o + 2) % 4] = (int)(self.z / 10) + 4
                    data[(self.o + 1) % 4] = 1
                    data[(self.o + 3) % 4] = 1
                    maz.maze[(self.x, self.y, self.z + 5)] = data
                    maz.count += 1
                    maz.rampcount += 1
                    maz.ramps.append((self.x, self.y, self.z + 5))
                    return -1
                if self.z % 10 == 0 and (self.x, self.y, self.z + 5) in maz.maze and (self.x, self.y, self.z + 5) not in maz.visited_ramps:
                    maz.count -= 1
                    maz.rampcount -= 1
                    maz.visited_ramps.append((self.x, self.y, self.z + 5))
                if self.z % 10 != 0 and (self.x, self.y, self.z) in maz.maze and (self.x, self.y, self.z) not in maz.visited_ramps:
                    maz.count -= 1
                    maz.rampcount -= 1
                    maz.visited_ramps.append((self.x, self.y, self.z))
            elif nextTileInfo[1] == 7:
                if (self.z % 10 == 0 and (self.x, self.y, self.z - 5) not in maz.maze) or (self.z % 10 != 0 and (self.x, self.y, self.z) not in maz.maze):
                    data = [0,0,0,0,2,7,0]
                    if robot.rampDir == -1:
                        data[(self.o + 2) % 4] = (int)(self.z / 10) + 4
                    data[(self.o + 1) % 4] = 1
                    data[(self.o + 3) % 4] = 1
                    maz.maze[(self.x, self.y, self.z - 5)] = data
                    maz.count += 1
                    maz.rampcount += 1
                    maz.ramps.append((self.x, self.y, self.z - 5))
                    return -1
                if self.z % 10 == 0 and (self.x, self.y, self.z - 5) in maz.maze and (self.x, self.y, self.z - 5) not in maz.visited_ramps:
                    maz.count -= 1
                    maz.rampcount -= 1
                    maz.visited_ramps.append((self.x, self.y, self.z - 5))
                if (self.x, self.y, self.z) and (self.x, self.y, self.z) in maz.maze not in maz.visited_ramps:
                    maz.count -= 1
                    maz.rampcount -= 1
                    maz.visited_ramps.append((self.x, self.y, self.z))
        else:
            if robot.rampDir == -1:
                if nextTileInfo[1] == 4:
                    maz.maze[(self.x, self.y, self.z)] = [1,1,1,1,1,0,0]
                    return -1
                else:
                    maz.coloured[(self.x, self.y, self.z)] = nextTileInfo[1]
        return 1
    def drop_cube(self):
        if (self.x, self.y, self.z) in maz.victims:
            victimInfo = maz.victims[(self.x, self.y, self.z)]
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
        print("---------- X: ", self.x, ", Y: ", self.y, ", Z: ", self.z, " ----------")
        print("---------- count: ", maz.count, ", rampcount: ", maz.rampcount, " ----------")
        info = self.Input()
        print(info)

        if info[5] == 3:
            time.sleep_ms(4000)

        self.drop_cube()
        if not self.path:
            if (self.x, self.y, self.z) not in maz.maze:
                if self.step == 0:
                    if info[2] != 1:
                        if maz.getCrds(2)[0] not in maz.maze and maz.getCrds(2)[1] not in maz.maze and \
                                maz.getCrds(2)[2] not in maz.maze:
                            maz.count += 1
                        else:
                            maz.count -= 1
                if info[3] != 1:
                    if maz.getCrds(3)[0] not in maz.maze and maz.getCrds(3)[1] not in maz.maze and maz.getCrds(3)[
                        2] not in maz.maze:
                        maz.count += 1
                    else:
                        maz.count -= 1
                if info[0] != 1:
                    if maz.getCrds(0)[0] not in maz.maze and maz.getCrds(0)[1] not in maz.maze and maz.getCrds(0)[
                        2] not in maz.maze:
                        maz.count += 1
                    else:
                        maz.count -= 1
                if info[1] != 1:
                    if maz.getCrds(1)[0] not in maz.maze and maz.getCrds(1)[1] not in maz.maze and maz.getCrds(1)[
                        2] not in maz.maze:
                        maz.count += 1
                    else:
                        maz.count -= 1
            if (self.x, self.y, self.z) not in maz.maze:
                maz.maze[(self.x, self.y, self.z)] = maz.save(info)
            maz.printMap()
            if info[0] != 1 and ((info[5] <= 5 and maz.getCrds(0)[0] not in maz.maze and maz.getCrds(0)[
                1] not in maz.maze and maz.getCrds(0)[2] not in maz.maze)):
                maz.count -= 1
                self.step += 1
                self.go(0)
            elif info[3] != 1 and (info[5] <= 5 and maz.getCrds(3)[0] not in maz.maze and maz.getCrds(3)[
                1] not in maz.maze and maz.getCrds(3)[2] not in maz.maze):
                maz.count -= 1
                self.step += 1
                self.go(3)
            elif info[1] != 1 and (info[5] <= 5 and maz.getCrds(1)[0] not in maz.maze and maz.getCrds(1)[
                1] not in maz.maze and maz.getCrds(1)[2] not in maz.maze):
                maz.count -= 1
                self.step += 1
                self.go(1)
            elif info[2] != 1 and (info[5] <= 5 and maz.getCrds(2)[0] not in maz.maze and maz.getCrds(2)[
                1] not in maz.maze and maz.getCrds(2)[2] not in maz.maze):
                maz.count -= 1
                self.step += 1
                self.go(2)
            else:
                if self.optimalPath() == -1:
                    return -1
                if self.path:
                    print(self.path[0])
                    self.step += 1
                    self.go(maz.getDir(self.path.pop(0)))
        else:
            print(self.path[0])
            maz.printMap()
            self.step += 1
            self.go(maz.getDir(self.path.pop(0)))
        if self.x == 0 and self.y == 0 and self.z == 0 and maz.count == 0:
            return 1
        return 0
    def optimalPath(self):
        print("finding optimal path")
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
        minCost = 2000
        for i in dest:
            if shortest in maz.ramps and i not in maz.ramps:
                minCost = cost[i]
                shortest = i
            elif cost[i] < minCost:
                minCost = cost[i]
                shortest = i
        if shortest in paths:
            self.path = paths[shortest]
        else:
            return -1
        for p in self.path:
            print(p, end=" ")
        print()
    def go(self, d):
        self.o = (self.o + d) % 4
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
class Maze:
    def __init__(self):
        self.maze = {}
        self.coloured = {}
        self.victims = {}
        self.count = 0
        self.rampcount = 0
        self.ramps = []
        self.visited_ramps = []
    def neighbour(self, tile, d):
        if d == 0:
            if (self.maze[tile][5] <= 5):
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
            if (self.maze[tile][5] <= 5):
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
            if (self.maze[tile][5] <= 5):
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
            if (self.maze[tile][5] <= 5):
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
        if n == 1:
            if robot.rampDir == -1:
                return (navi.x + 1, navi.y, navi.z), (navi.x + 1, navi.y, navi.z + 5), (navi.x + 1, navi.y, navi.z - 5)
        if n == 2:
            if robot.rampDir == -1:
                return (navi.x, navi.y - 1, navi.z), (navi.x, navi.y - 1, navi.z + 5), (navi.x, navi.y - 1, navi.z - 5)
        if n == 3:
            if robot.rampDir == -1:
                return (navi.x - 1, navi.y, navi.z), (navi.x - 1, navi.y, navi.z + 5), (navi.x - 1, navi.y, navi.z - 5)
    def getDir(self, d):
        return (d - navi.o + 4) % 4
    def printMap(self):
        pass
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
def flash(count = 5):
    for i in range(count):
        p1.value(1)
        time.sleep_ms(500)
        p1.value(0)
        time.sleep_ms(500)
while True:
    print("\n\n\nstart...")
    p1.value(1)
    robot = Robot()
    navi = Nav()
    maz = Maze()

    while True:
        robot.cam.img = sensor.snapshot()
        v = robot.motor.voltage()
        print('voltage: ', v)
        if v > 7: break
        p1.value(1)
        time.sleep_ms(100)
        p1.value(0)
        time.sleep_ms(100)
    for i in range(3):
        p1.value(1)
        time.sleep_ms(500)
        p1.value(0)
        time.sleep_ms(500)
    try:
        restart = False
        while True:
            result = navi.nav()
            gc.collect()
            time.sleep_ms(10)
            if result == -1:
                robot.motor.stop()
                restart = True
            if result == 1 or result == -1 or navi.step > 1000: break
            if button_pressed == 1:
                robot.motor.stop()
                proc_button()
                restart = True
                break
        if restart: continue
        flash_blue()
    except KeyboardInterrupt:
        servo9.set(ms=1.5)
    finally:
        servo9.set(ms=1.5)
        robot.motor.stop()
    print("finished")
