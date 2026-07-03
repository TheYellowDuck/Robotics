# SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
# Required Notice: Copyright (c) 2026 George Zhang — https://github.com/TheYellowDuck

# from pyb2 import *
import time, math

# abcdefg                     0 = no, 1 = yes
# a = forward wall
# b = right wall
# c = back wall
# d = left wall
# e = tile weight
# f = tile type
# g = tile specifics

SX = 4
SY = 5
SZ = 1

W = 9
L = 9
H = 2

class Robot:
    def __init__(self):
        self.inputType = 0
        # self.ld = Lidar(3)
        # self.motor = SServo(1)
        self.anglePrecision = 6.5
        self.rampDir = -1
        self.TestMaze = [[]]
        self.X = SX
        self.Y = SY
        self.Z = SZ
        if self.inputType == 0:
            f = open('TestMaze3.txt','r')
            # f = -1
            l = 0
            for line in f.readlines():
                line = line[:-1]
                if line[0] == 'L':
                    l = int(line[1]) - 1
                    self.TestMaze.append([])
                else:
                    self.TestMaze[l].append(line.split(' '))
            #     if len(line) == 0:
            #         self.TestMaze.append([])
            #         l += 1
            #     # print(line)
            #     self.TestMaze[l].append(line.split(' '))
            # # print()
            for i in self.TestMaze:
                for j in i:
                    for k in j:
                        print(k, end=' ')
                    print()
                print()
            print()
    def Input(self):
        if self.inputType == 0:
            return self.Sim()
        else:
            return self.Sensor()
    def Sim(self):
        # lz = SZ * 10 + self.Z
        # issue here {
        # if lz % 10 != 0:
        #     lz == SZ * 10 + navi.prev
        # if navi.prev % 10 == 0:
        #     if navi.ramp == 1:
        #         lz += 5
        #     if navi.ramp == 0:
        #         lz -= 5
        # else

        # }
        print(self.X, self.Y, self.Z)
        # lz = round(lz / 10)
        # print(self.X, self.Y, lz)
        # input = [int(i) for i in str(self.TestMaze[lz][SY - self.Y][SX + self.X])]
        input = [int(i) for i in str(self.TestMaze[self.Z][self.Y][self.X])]
        if len(input) == 4:
            input.append(1)
        if len(input) == 5:
            input.append(0)
        if len(input) == 6:
            input.append(0)
        # if len(input) == 7:
        #     input.append(0)
        # if input[5] != 1 and navi.z % 10 != 0:
        #     if navi.prev > navi.z:
        #         lz = SZ * 10 + navi.prev - 10
        #     else:
        #         lz = SZ * 10 + navi.prev + 10
        #     lz = round(lz / 10)
        #     input = [int(i) for i in str(self.TestMaze[lz][SY - navi.y][SX + navi.x])]
        #     if len(input) == 4:
        #         input.append(1)
        #     if len(input) == 5:
        #         input.append(0)
        #     if len(input) == 6:
        #         input.append(0)
        print(input)

        for i in range(4):
            if input[i] > 1:
                input[i] = 0

        if navi.o == 1:
            t = input[0]
            input[0] = input[1]
            input[1] = input[2]
            input[2] = input[3]
            input[3] = t
        if navi.o == 2:
            t = input[0]
            input[0] = input[2]
            input[2] = t
            t = input[1]
            input[1] = input[3]
            input[3] = t
        if navi.o == 3:
            t = input[0]
            input[0] = input[3]
            input[3] = input[2]
            input[2] = input[1]
            input[1] = t
        if input[5] == 1:
            if self.Z < input[6]:
                input[6] = 1
            elif self.Z > input[6]:
                input[6] = 0
            else:
                input[6] = 0
                self.rampDir = -1
        else:
            input[6] = 0
            self.rampDir = -1
        if input[5] == 1:
            self.rampDir = input[6]
        else:
            self.rampDir = -1
        return input
    def Sensor(self):
        #dist = self.ld.read()
        dist = self.getData()
        print(dist.tolist())
        ds = mins(dist)
        print(ds)
        angle = self.getAngle(ds)
        print(angle)
        time.sleep_ms(5000)
        #if abs(angle) > self.anglePrecision:
            #self.fixAngle(angle)
        walls = self.checkWalls(angle, dist)
        if len(walls) == 4:
            walls.append(1)
        if len(walls) == 5:
            walls.append(0)
        if len(walls) == 6:
            walls.append(0)
        if walls[5] == 1:
            if self.Z < walls[6]:
                walls[6] = 1
            elif self.Z > walls[6]:
                walls[6] = 0
            else:
                walls[6] = 0
                self.rampDir = -1
        else:
            walls[6] = 0
            self.rampDir = -1
        if walls[5] == 1:
            self.rampDir = walls[6]
        else:
            self.rampDir = -1
        print(walls)
        return walls
        # pass
    def getData(self):
        dist = np.array(self.ld.read(), dtype = np.float)
        ds = filter(dist, [3, 10])
        return ds
    def getAngle(self, ds):
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

    def turn(self, angle, d):
        speed = 1000
        count = 0
        t = 0
        if d == 1:
            t = abs(round((90 - angle) * self.anglePrecision * self.anglePrecision))
            self.motor.set_speeds([-speed, -speed, -speed, -speed])
        if d == 2:
            t = abs(round((180 - abs(angle)) * self.anglePrecision * self.anglePrecision))
            if angle > self.anglePrecision:
                self.motor.set_speeds([speed, speed, speed, speed])
            if angle < self.anglePrecision:
                self.motor.set_speeds([-speed, -speed, -speed, -speed])
        if d == 3:
            t = abs(round((-90 - angle) * self.anglePrecision * self.anglePrecision))
            self.motor.set_speeds([speed, speed, speed, speed])
        #print(angle, t)
        time.sleep_ms(t)
        self.motor.set_speeds([0, 0, 0, 0])
        #pass

    # def fixPos(self, angle):
    #     center = round(angle)
    #     distance1 = 0.0
    #     r = 30
    #     for i in range(center - r, center + r - 1):
    #         #print(j)
    #         distance1 += ds[j] * abs(math.sin(math.radians(i - angle)))
    #     distance1 /= 2 * r
    #     distance1 %= 30
    #     center = round(angle + 2 * 90)
    #     distance2 = 0.0
    #     for i in range(center - r, center + r - 1):
    #         #print(j)
    #         distance2 += ds[j] * abs(math.sin(math.radians(i - angle)))
    #     distance2 /= 2 * r
    #     distance2 %= 30


    def fixAngle(self, angle):
        t = abs(round(angle * self.anglePrecision / 4))
        speed = 1000
        count = 0
        while abs(angle) > self.anglePrecision and count < 3:
            if angle > self.anglePrecision:
                self.motor.set_speeds([-speed, -speed, -speed, -speed])
                #adjust right
            if angle < self.anglePrecision:
                #adjust left
                self.motor.set_speeds([speed, speed, speed, speed])
            time.sleep_ms(t)
            #t /= 2
            #t = round(t)
            #speed /= 2
            #speed = round(speed)
            #speed = round(angle * 100)

            #dist = self.ld.read()
            dist = self.getData()
            ds = mins(dist)
            print(ds)
            angle = self.getAngle(ds)
            t = abs(round(angle * self.anglePrecision / 4))
            print(angle)
            count += 1
            #pass
        self.motor.set_speeds([0, 0, 0, 0])

    def checkWalls(self, angle, ds):
        walls = [0,0,0,0]
        for i in range(4):
            #center = round(angle + i * 90)
            center = round(i * 90)
            #distance = 0.0
            #r = 30
            #for j in range(center - r, center + r - 1):
                ##print(j)
                #distance += ds[j] * abs(math.sin(math.radians(j - angle)))
            #distance /= 2 * r
            distance = ds[center]
            print(distance)
            if distance <= 250:
                walls[(i + 1) % 4] = 1
        return walls

    def move(self, d):
        t = 3000
        speed = 2000
        if d == 0:
            self.motor.set_speeds([-speed, -speed, speed, speed])
        else:
            self.motor.set_speeds([speed, speed, -speed, -speed])
        time.sleep_ms(t)
        self.motor.set_speeds([0, 0, 0, 0])

    def go(self, d):
        if self.inputType == 0:
            cur = [int(i) for i in str(self.TestMaze[self.Z][self.Y][self.X])]
            if len(cur) == 4:
                cur.append(1)
            if len(cur) == 5:
                cur.append(0)
            if len(cur) == 6:
                cur.append(0)
            # navi.o = (navi.o + d) % 4
            if cur[5] == 1 and cur[navi.o] != 0:
                print("GO:", cur)
                self.Z = cur[navi.o] - 2
            # if self.rampDir != -1:
            #     if self.rampDir == 0:
            #         navi.z -= 0.5
            #     else:
            #         navi.z += 0.5
            if navi.o == 0:
                self.Y -= 1
            if navi.o == 1:
                self.X += 1
            if navi.o == 2:
                self.Y += 1
            if navi.o == 3:
                self.X -= 1

        if self.inputType == 1:
            #dist = self.ld.read()
            dist = self.getData()
            ds = mins(dist)
            print(ds)
            angle = self.getAngle(ds)
            print(angle)
            self.turn(angle, d)
            self.move(0)

    # def goRamp(self, d):
    #     if d == 0:
    #         self.z -= 1
    #     else:
    #         self.z += 1
    # pass

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
        self.prevT = (0,0,0)
        # self.count = 0
        self.path = []
    def nav(self):
        info = robot.Input()
        # print(info)
        if info[5] == 1:
            if self.z % 10 == 0:
                if robot.rampDir == 0:
                    self.z -= 5
                else:
                    self.z += 5
                info[((self.o + 2) % 4 - self.o + 4) % 4] = (int) (self.prev / 10) + 4
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
                    self.prevT[self.o] = (int) (self.z / 10) + 4
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
                            if maz.getCrds(2)[0] not in maz.maze and maz.getCrds(2)[1] not in maz.maze and maz.getCrds(2)[2] not in maz.maze:
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
                        if maz.getCrds(3)[0] not in maz.maze and maz.getCrds(3)[1] not in maz.maze and maz.getCrds(3)[2] not in maz.maze:
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
                        if maz.getCrds(0)[0] not in maz.maze and maz.getCrds(0)[1] not in maz.maze and maz.getCrds(0)[2] not in maz.maze:
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
                        if maz.getCrds(1)[0] not in maz.maze and maz.getCrds(1)[1] not in maz.maze and maz.getCrds(1)[2] not in maz.maze:
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
                if info[3] != 1 and ((info[5] == 0 and maz.getCrds(3)[0] not in maz.maze and maz.getCrds(3)[1] not in maz.maze and maz.getCrds(3)[2] not in maz.maze) or (info[5] == 1 and maz.getCrds(3)[0] not in maz.maze and maz.getCrds(3)[1] not in maz.maze)):
                    maz.count -= 1
                    if info[5] == 1:
                        maz.rampcount -= 1
                    self.step += 1
                    # self.pd = 1
                    self.go(3)
                elif info[0] != 1 and ((info[5] == 0 and maz.getCrds(0)[0] not in maz.maze and maz.getCrds(0)[1] not in maz.maze and maz.getCrds(0)[2] not in maz.maze) or (info[5] == 1 and maz.getCrds(0)[0] not in maz.maze and maz.getCrds(0)[1] not in maz.maze)):
                    maz.count -= 1
                    if info[5] == 1:
                        maz.rampcount -= 1
                    self.step += 1
                    # self.pd = 2
                    self.go(0)
                elif info[1] != 1 and ((info[5] == 0 and maz.getCrds(1)[0] not in maz.maze and maz.getCrds(1)[1] not in maz.maze and maz.getCrds(1)[2] not in maz.maze) or (info[5] == 1 and maz.getCrds(1)[0] not in maz.maze and maz.getCrds(1)[1] not in maz.maze)):
                    maz.count -= 1
                    if info[5] == 1:
                        maz.rampcount -= 1
                    self.step += 1
                    # self.pd = 3
                    self.go(1)
                elif info[2] != 1 and ((info[5] == 0 and maz.getCrds(2)[0] not in maz.maze and maz.getCrds(2)[1] not in maz.maze and maz.getCrds(2)[2] not in maz.maze) or (info[5] == 1 and maz.getCrds(2)[0] not in maz.maze and maz.getCrds(2)[1] not in maz.maze)):
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
        print(self.x, self.y, self.z)
        queue = []
        queue.append((self.x, self.y, self.z))
        weights = {}
        weights[(self.x, self.y, self.z)] = 0
        paths = {}
        paths[(self.x, self.y, self.z)] = []
        dest = []
        while queue:
            cur = queue.pop(0)
            for i in range(0, 4):
                if maz.maze[cur][i] != 1:
                    next = maz.neighbour(cur, i)
                    if next != -1:
                        if next in weights:
                            if (weights[next] > weights[cur] + maz.maze[next][4]):
                                weights[next] = weights[cur] + maz.maze[next][4]
                                paths[next] = paths[cur].copy()
                                paths[next].append(i)
                                queue.append(next)
                        else:
                            weights[next] = weights[cur] + maz.maze[next][4]
                            paths[next] = paths[cur].copy()
                            paths[next].append(i)
                            queue.append(next)
                    else:
                        dest.append(cur)
        shortest = (0, 0, 0)
        weight = 2000000000
        for i in dest:
            if shortest in maz.ramps and i not in maz.ramps:
                weight = weights[i]
                shortest = i
            elif weights[i] < weight:
                weight = weights[i]
                shortest = i
        self.path = paths[shortest]
        for p in self.path:
            print(p, end = " ")
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
        # pass
        if robot.inputType == 0:
            print(navi.x, navi.y, navi.z, navi.o)
            print(robot.X, robot.Y, robot.Z)
            print(self.count, self.rampcount, navi.step)
            for k in range (-SZ, H-SZ):
                for i in range(SY, SY-L, -1):
                    for j in range(-SX, W-SX):
                        if j == navi.x and i == navi.y and (k * 10 == navi.z or k * 10 + 5 == navi.z or k * 10 - 5 == navi.z):
                            print('$robot$', end = ", ")
                        else:
                            if (j, i, k * 10) in self.maze:
                                print(''.join(str (e) for e in self.maze[(j, i, k * 10)]), end = ", ")
                            elif (j, i, k * 10 + 5) in self.maze:
                                print(''.join(str (e) for e in self.maze[(j, i, k * 10 + 5)]), end = ", ")
                            elif (j, i, k * 10 - 5) in self.maze:
                                print(''.join(str (e) for e in self.maze[(j, i, k * 10 - 5)]), end = ", ")
                            else:
                                print('*******', end = ", ")
                    print()
                print()
            print()
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
    # pass
    def rampLev(self, l, d):
        if d == 0:
            return l - 0.5
        else:
            return l + 0.5
    def black(self, a, b):
        self.maze[(a, b)] = [1,1,1,1,1]
        pass


robot = Robot()
navi = Nav()
maz = Maze()

# m = Maze()
while True:
    if navi.nav() == 1:
        break
    ## time.sleep(0.5)
#robot.Input()
print(navi.x, navi.y, navi.y, navi.step)
maz.printMap()
