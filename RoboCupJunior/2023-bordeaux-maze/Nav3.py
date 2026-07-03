# SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
# Required Notice: Copyright (c) 2026 George Zhang — https://github.com/TheYellowDuck

from pyb2 import *
import time, math
import sys
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


from pyb2 import *
import time, math
import sys

from pyb import millis, LED


# use sts3215
# used by robot
class Motor:
    def __init__(self):
        print("create new motor")
        self.motor = SServo('LP1', [11, 12, 21, 22])

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
    def forward_ms(self, speed, ms):
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
            p1 = self.motor.get_position(2)
            p2 = (p1 - p0) % 4096
            if last_p2 == 0 and p2 > 4090:
                p2 = 0
                flag = 1
            if p2 < 2048 and last_p2 > 2048: c += 1
            last_p2 = p2
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

    def voltage(self):
        v = self.motor.voltage()
        return v


class Robot():
    def __init__(self):
        self.motor = Motor()
        self.lidar = Lidar(4)
        self.lidar_data = LidarData([])

        self.ldata = []

        self.rampDir = -1

    def check_walls(self):
        self.get_lidardata()
        # print(self.ck_walls)
        return self.ck_walls[0]

    def forward(self):
        self.get_lidardata()
        # print(self.ck_walls)
        if self.ck_walls[0][2] == 1 or self.ck_walls[1][2][1] == 1:
            self.forward_reference_back_dist()
        else:
            self.motor.forward_ms(2500, 2400)
        self.adjust_angle()

    def adjust_angle(self):
        self.get_lidardata()
        print(self.ld_lines)
        print(self.ld_ang)
        if abs(self.ld_ang) <= 5:
            return
        if self.ld_ang > 0:
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
        else:
            self.motor.stop()
            flash_red(5)
            SystemExit
        self.motor.stop()

    def right(self):
        self.motor.turn_right()
        self.forward()
    def back(self):
        self.motor.turn_back();
        self.forward()
    def left(self):
        self.motor.turn_left()
        self.forward()

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
        for m,v in self.ld_lines:
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
            self.ld_ang = -999

    def forward_reference_back_dist (self):
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

    def go(self, d):
        if (d == 0): #go forwards
            self.forward()
        if (d == 1): #go right
            self.right()
        if (d == 2): #go back
            self.back()
        if (d == 3): #go left
            self.left()


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

        #task 3: follow wall to next tile
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

        #task 4: implement one function to control robot
        # print(robot.motor.voltage())
        # robot.go(0)
        # time.sleep_ms(1000)
        # robot.go(1)
        # time.sleep_ms(1000)
        # robot.go(2)
        # time.sleep_ms(1000)
        # robot.go(3)
        # time.sleep_ms(1000)

        #task 5: add an input function for nav
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
        # self.count = 0
        self.path = []

    def Input(self):
        input = robot.check_walls()
        if len(input) == 4:
            input.append(1)
        if len(input) == 5:
            input.append(0)
        if len(input) == 6:
            input.append(0)
        if input[5] == 1:
            if self.Z < input[6]:
                input[6] = 1
            elif self.Z > input[6]:
                input[6] = 0
            else:
                input[6] = 0
                robot.rampDir = -1
        else:
            input[6] = 0
            robot.rampDir = -1
        if input[5] == 1:
            robot.rampDir = input[6]
        else:
            robot.rampDir = -1
        return input

    def nav(self):
        # clock.tick()
        info = self.Input()

        #if (self.x, self.y, self.z) in maz.maze:
            #newinfo = maze.save(info)
            #same = 1
            #for i in range(4):
                #if newinfo[i] != maze.maze[(self.x, self.y, self.z)][i]:
                    #same = 0
                    #break
            #if same == 0:
                #pass


        # print('robot input fps: ', clock.fps())
        # print(info)
        if info[5] == 1:
            if self.z % 10 == 0:
                if robot.rampDir == 0:
                    self.z -= 5
                else:
                    self.z += 5
                info[((self.o + 2) % 4 - self.o + 4) % 4] = (int)(self.prev / 10) + 4
            # info[6] == round(self.z - 0.5, ndigits=1)
            # info[7] == round(self.z + 0.5, ndigits=1)
        else:
            if self.ramp != -1:
                temp = self.z
                if self.ramp == 0:
                    self.z -= 5
                if self.ramp == 1:
                    self.z += 5
                if (self.x, self.y, self.z) not in maz.maze:
                    self.prevT[self.o] = (int)(self.z / 10) + 4
                    if self.o == 0:
                        maz.maze[(self.x, self.y - 1, temp)] = self.prevT
                    if self.o == 1:
                        maz.maze[(self.x - 1, self.y, temp)] = self.prevT
                    if self.o == 2:
                        maz.maze[(self.x, self.y + 1, temp)] = self.prevT
                    if self.o == 3:
                        maz.maze[(self.x + 1, self.y, temp)] = self.prevT
        self.prevT = maz.save(info)
        print(info)
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
        if robot.rampDir == -1:
            self.ramp = -1
        else:
            if back == 0:
                self.ramp = robot.rampDir
            elif robot.rampDir == 0:
                self.ramp = 1
            else:
                self.ramp = 0
        if self.z % 10 == 0:
            self.prev = self.z
        # self.prevT = maz.maze[(self.x, self.y, self.z)]
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
        go_ramp = 1
        for i in dest:
            if go_ramp == 0 and i in self.maze.ramps:
                continue
            if go_ramp == 0 and i not in self.maze.ramps and cost[i] < minCost:
                minCost = cost[i]
                shortest = i
            if go_ramp == 1 and shortest in self.maze.ramps and i not in self.maze.ramps:
                go_ramp = 0
                minCost = cost[i]
                shortest = i
            if go_ramp == 1 and shortest in self.maze.ramps and i in self.maze.ramps and cost[i] < minCost:
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
        robot.go(d)
        #log.debug('go: ' + str(d))
    # pass
class Maze:
    def __init__(self):
        self.maze = {}
        self.count = 0
        self.rampcount = 0
        self.ramps = []
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
        #if z % 10 == 0:

     pass
    def rampLev(self, l, d):
        if d == 0:
            return l - 0.5
        else:
            return l + 0.5
    def black(self, a, b):
        self.maze[(a, b)] = [1, 1, 1, 1, 1]
        pass


def flash_blue(count=10000):
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

print("\n\n\nstart...")
robot = Robot()
navi = Nav()
maz = Maze()
flash_green(3)
# m = Maze()
try:
    #robot.fixPos()
    #robot.go(0)

    # while True:
    #     if navi.nav() == 1:
    #         break
    #     time.sleep_ms(10)

    robot.adjust_angle()

    flash_blue()
except KeyboardInterrupt:
    print("Ctrl-C")
finally:
    #log.flush()
    # robot.motor2.stop()
    robot.motor.stop()
print("finished")
# robot.Input()
print(navi.x, navi.y, navi.y, navi.step)
maz.printMap()
