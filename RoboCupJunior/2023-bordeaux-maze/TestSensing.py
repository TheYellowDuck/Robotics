from pyb import millis, LED
from machine import Pin
from pyb2 import *
import sensor, image, time, math
import sys

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
blue_threshold = (0, 100, -5, 21, -128, -22)  # L A B

#bw = [(0, 20, -23, 25, -23, 25)]
black_threshold = (21, 38, -5, 13, -7, 11)
white_threshold = (40, 100, -18, 2, -8, 32)

letter_threshold = (17, 60, -3, 8, -9, 5)
class Camera():
    def __init__(self):
        self.img = sensor.snapshot()
        self.tempCount = -1
        self.tempCount2 = -1

    def scan(self):
        self.img = sensor.snapshot()
        self.img.gaussian(1)
        time.sleep_ms(100)

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
        temp = [self.check_green(), self.check_yellow(), self.check_red(), self.check_letter()]
        print(temp)
        for i in range(4):
            for j in range(4):
                if temp[j][i] != 0:
                    arr[i] = temp[j][i]
                    break
        # arr[self.tempCount2] = 10 + self.tempCount
        return arr

    def check_tile(self):
        self.scan()
        #1: white
        #2: silver
        #3: blue
        #4: black
        v = self.check_blue()
        if v == 1:
            v = self.check_black()
        return v

    def check_red(self):
        print("RED")
        #filtered = self.img
        #filtered.binary([red_threshold])
        #self.img.binary([red_threshold])
        return self.get_color_wall(12)
    def check_yellow(self):
        print("YELLOW")
        filtered = self.img
        #filtered.binary([yellow_threshold])
        return self.get_color_wall(11)
    def check_green(self):
        print("GREEN")
        filtered = self.img
        #filtered.binary([green_threshold])
        return self.get_color_wall(10)
    def check_blue(self):
        print("BLUE")
        filtered = self.img
        #filtered.binary([blue_threshold])
        return self.get_color_ground(3)
    def check_black(self):
        print("BLACK")
        filtered = self.img
        #filtered.binary(bw)
        return self.get_color_ground(4)
    def check_silver(self):
        print("SILVER")
        pass

    def get_color_wall(self):
        w = 320
        h = 240
        fov = 100

        arr = [0,0,0,0]

        robot.get_lidardata()

        self.img.draw_line(0, (int)(h/2) - 30, w, (int)(h/2) - 30)

        for blob in self.img.find_blobs([red_threshold, green_threshold, yellow_threshold], pixels_threshold=325, area_threshold=325, merge=False):
            if blob.y() >= (int) (h/2) - 30 and blob.area() <= 5000:
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

        w = 320
        h = 240

        robot.get_lidardata()

        #self.img.draw_rectangle(((int) (w/8), (int) (h/2) + 30, (int) (w*6/8), h - ((int)(h/2) - 30)))

        for blob in self.img.find_blobs([white_threshold, blue_threshold, black_threshold], pixels_threshold=1500, area_threshold=1500, merge=False, roi = ((int) (w/8), (int) (h/2) + 30, (int)(w*6/8), h - ((int) (h/2) + 30))):
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

        pixelcount = 0
        for i in range(blob.cy() - 5, blob.cy() + 10):
            pixel = self.img.get_pixel(blob.cx(), i)
            pixel = image.rgb_to_lab(pixel)
            if self.check_threshold(pixel) == 1:
                pixelcount += 1
            print(pixel)
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

        w = 320
        h = 240
        fov = 100

        arr = [0,0,0,0]

        robot.get_lidardata()

        #self.img.binary([letter_threshold])

        #self.img.draw_rectangle((0, (int) (h/2) - 30, w, (int)(h/3) + 10))

        for blob in self.img.find_blobs([letter_threshold], pixels_threshold=50, area_threshold=200, merge=True, roi = (0, (int) (h/2) - 30, w, (int)(h/3) + 10)):
            if blob.area() <= 5000 and blob.h() >= blob.w() - 5:
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
        self.cam = Camera()
        self.lidar = -1
        self.lidar_data = -1
        #self.lidar = Lidar(3)
        #self.lidar_data = LidarData([])
        self.ldata = []
        self.rampDir = -1
        self.wall_ref = 150
    def check_wall_reference(self):
        data = self.lidar.read()
        self.lidar_data.data = data
        ds = self.lidar_data.get_distance([0, 90, 180, 270])
        print(ds)
        m = min(ds)
        self.wall_ref = m
    def check_walls(self):
        self.get_lidardata()
        # print(self.ck_walls)
        return self.ck_walls[0]
    def check_tile_info(self):
        tile = self.check_ramp() + 4
        if tile == 4:
            tile = self.cam.check_tile()
        return [self.cam.check_victims(), tile]
    def check_ramp(self):
        self.get_lidardata()
        front = self.lidar_data.data[0]
        if front > 250 and front < 350:
            return 1
        return 0
    def get_lidardata(self):
        self.ld_ang = 0
        #self.ldata = self.lidar.read(timeout=3000, percent=100)
        #self.ldata = np.array(self.ldata, dtype=np.float)
        #self.lidar_data.data = self.ldata
        #self.ck_walls = self.lidar_data.check_walls()
        #self.ld_lines = self.lidar_data.find_lines()
        #cnt = 0
        #sum = 0
        #self.ld_ang = 0
        #for m, v in self.ld_lines:
            #if v < 5:
                #mm = m
                #if m < -45:
                    #mm += 90
                #elif m > 45:
                    #mm -= 90
                ## print(m, mm)
                #cnt += 1
                #sum += mm
            ## else:
            ##     print('bad data', m, v)
        #if cnt > 0:
            #self.ld_ang = sum / cnt
        #else:
            #self.ld_ang = 0
while True:
    print("\n\n\nstart...")
    robot = Robot()
    try:
        robot.cam.scan()
        while True:
            robot.cam.scan()
            print(robot.cam.get_color_wall())
            #print(robot.cam.get_color_ground())
            #print(robot.cam.get_letter())
        flash_blue()
    except KeyboardInterrupt:
        print("Ctrl-C")
    print("finished")
