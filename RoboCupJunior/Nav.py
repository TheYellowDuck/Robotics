# SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
# Required Notice: Copyright (c) 2026 George Zhang — https://github.com/TheYellowDuck

import time
# TestMaze = [[11101110, 11001110, 11000110, 11000110, 11000110, 11000110, 11000110, 11100110],
#         [10101110, 10101110, 10001110, 10010110, 10010110, 10100110, 10001110, 10100110],
#         [10101110, 10111110, 10101110, 11001110, 11000110, 10100110, 10001110, 10100110],
#         [10001110, 11010110, 10100110, 10001110, 10000110, 10100110, 10011110, 10100110],
#         [10001110, 11100110, 10001110, 10000110, 10000110, 10000110, 11010110, 10100110],
#         [10001110, 10100110, 10001110, 10000110, 10010110, 10100110, 11111110, 10101110],
#         [10001110, 10000110, 10000110, 10110110, 11111110, 10001110, 11000110, 10100110],
#         [10011110, 10010110, 10010110, 11010110, 11010110, 10010110, 10010110, 10110110]]
TestMaze1 = [[[11101110, 11001110, 11000110, 11000110, 11000110, 11000110, 11000110, 11100110, 11111110],
        [10101110, 10101110, 10001110, 10010110, 10010110, 10100110, 10001110, 10000110, 11100110],
        [10101110, 10111110, 10101110, 11001110, 11000110, 10100110, 10001110, 10100110, 10101110],
        [10001110, 11010110, 10100110, 10001110, 10000110, 10100110, 10011110, 10100110, 10101110],
        [10001110, 11100110, 10001110, 10000110, 10000110, 10000110, 11010110, 10100110, 10100100],
        [10001110, 10100110, 10001110, 10000110, 10010110, 10100110, 11111110, 10101110, 11111110],
        [10001110, 10000110, 10000110, 10110110, 11111110, 10001110, 11000110, 10100110, 11111110],
        [10011110, 10010110, 10000110, 11010110, 11010110, 10010110, 10010110, 10110110, 11111110],
        [11111110, 11111110, 10011110, 10110100, 11111110, 11111110, 11111110, 11111110, 11111110]],
        [[11101110, 11001110, 11000110, 11000110, 11000110, 11000110, 11000110, 11100110, 11111110],
        [10101110, 10101110, 10001110, 10010110, 10010110, 10100110, 10001110, 10000110, 11100110],
        [10101110, 10111110, 10101110, 11001110, 11000110, 10100110, 10001110, 10100110, 10101110],
        [10001110, 11010110, 10100110, 10001110, 10000110, 10100110, 10011110, 10100110, 10101110],
        [10001110, 11100110, 10001110, 11001210, 11000210, 11010210, 11000210, 11000210, 10100010],
        [10001110, 10100110, 10001110, 10001210, 10110210, 11111210, 10001210, 10110210, 11111110],
        [10001110, 10000110, 10000110, 10101210, 11001210, 11000210, 10100210, 11101210, 11111110],
        [10011110, 10010110, 10000110, 10101210, 10011210, 10010210, 10010210, 10110210, 11111110],
        [11111110, 11111110, 10011110, 10110010, 11111110, 11111110, 11111110, 11111110, 11111110]]]
# TestMaze2 = [[11001210, 11000210, 11010210, 11000210, 11000210],
#              [10001210, 10110210, 11111210, 10001210, 10110210],
#              [10101210, 11001210, 11000210, 10100210, 11101210],
#              [10101210, 10011210, 10010210, 10010210, 10110210]]
zx = 0
zy = 7
curlevel = 0
def Robot(ax, ay, az, o):
    input = [int(i) for i in str(TestMaze1[curlevel + az][zy - ay][zx + ax])]
    if o == 1:
        input[0] = input[1]
        input[1] = input[2]
        input[2] = input[3]
        input[3] = input[4]
        input[4] = input[0]
        input[0] = 1
    if o == 2:
        input[0] = input[1]
        input[1] = input[3]
        input[3] = input[0]
        input[0] = input[2]
        input[2] = input[4]
        input[4] = input[0]
        input[0] = 1
    if o == 3:
        input[0] = input[1]
        input[1] = input[4]
        input[4] = input[3]
        input[3] = input[2]
        input[2] = input[0]
        input[0] = 1
    return input
    # pass

# 0 = front
# 1 = right
# 2 = back
# 3 = left

# 0 = blank tile
# -3 = black tile
# -2 = blue tile
# -1 = obstacle
# 1 = ramp
# 2 = checkpoint
# 3 = victim

class Maze:
    def __init__(self):
        self.maze = {}
        self.o = 0
        self.x = 0
        self.y = 0
        self.z = 0
        self.step = 0
        self.count = 0
        self.path = []
    def neighbour(self, a, b, d):
        if d == 0:
            return (a, b + 1, self.z)
        if d == 1:
            return (a + 1, b, self.z)
        if d == 2:
            return (a, b - 1, self.z)
        if d == 3:
            return (a - 1, b, self.z)
    def getCrds(self, d):
        n = (self.o + d) % 4
        if n == 0:
            return (self.x, self.y + 1)
        if n == 1:
            return (self.x + 1, self.y)
        if n == 2:
            return (self.x, self.y - 1)
        if n == 3:
            return (self.x - 1, self.y)
    def getDir(self, d):
        return (d - self.o + 4) % 4
    def printMap(self):
        print(self.x, self.y, self.z, self.count)
        for i in range(zy, zy-9, -1):
            for j in range(-zx, 9-zx):
                if j == self.x and i == self.y and 0 == self.z:
                    print('$robot$', end = ", ")
                else:
                    if (j, i, 0) in self.maze:
                        print(''.join(str (e) for e in self.maze[(j, i, 0)]), end = ", ")
                    else:
                        print('*******', end = ", ")
            print("          ", end = " ")
            for j in range(-zx, 9-zx):
                if j == self.x and i == self.y and 1 == self.z:
                    print('$robot$', end = ", ")
                else:
                    if (j, i, 1) in self.maze:
                        print(''.join(str (e) for e in self.maze[(j, i, 1)]), end = ", ")
                    else:
                        print('*******', end = ", ")
            print()
        print()
    def nav(self):
        info = Robot(self.x, self.y, self.z, self.o)
        info.pop(0)
        if not self.path:
            if info[5] != 0:
                if (self.x, self.y, self.z) not in self.maze:
                    if self.step == 0:
                        if info[2] == 0:
                            if self.getCrds(2) not in self.maze:
                                self.count += 1
                            else:
                                self.count -= 1
                    if info[3] == 0:
                        if self.getCrds(3) not in self.maze:
                            self.count += 1
                        else:
                            self.count -= 1
                    if info[0] == 0:
                        if self.getCrds(0) not in self.maze:
                            self.count += 1
                        else:
                            self.count -= 1
                    if info[1] == 0:
                        if self.getCrds(1) not in self.maze:
                            self.count += 1
                        else:
                            self.count -= 1
                    # if info[2] == 0:
                    #     if self.getCrds(2) not in self.maze:
                    #         self.count += 1
                    #     else:
                    #         self.count -= 1
                self.maze[(self.x, self.y, self.z)] = self.save(info)
                        
                self.printMap()
                if info[3] == 0 and self.getCrds(3) not in self.maze:
                    self.count -= 1
                    self.step += 1
                    # self.pd = 1
                    go(self, 3)
                elif info[0] == 0 and self.getCrds(0) not in self.maze:
                    self.count -= 1
                    self.step += 1
                    # self.pd = 2
                    go(self, 0)
                elif info[1] == 0 and self.getCrds(1) not in self.maze:
                    self.count -= 1
                    self.step += 1
                    # self.pd = 3
                    go(self, 1)
                elif info[2] == 0 and self.getCrds(2) not in self.maze:
                    self.count -= 1
                    self.step += 1
                    # self.pd = 0
                    go(self, 2)
                else:
                    self.dijkstra()
                    self.step += 1
                    if self.path:
                        go(self, self.getDir(self.path.pop(0)))
            else:
                self.maze[(self.x, self.y, self.z)] = self.save(info)
                if self.count == 0:
                    if self.z == 0:
                        self.goRamp(1)
                    else:
                        self.goRamp(-1) 
                else:
                    self.dijkstra()
                    self.step += 1
                    if self.path:
                        go(self, self.getDir(self.path.pop(0)))
        else:
            self.printMap()
            self.step += 1
            go(self, self.getDir(self.path.pop(0)))
        if self.x == 0 and self.y == 0 and self.z == 0 and self.count == 0:
            return 1
        return 0
    def goRamp(self, d):
        self.z += d
        # pass
    def save(self, info):
        out = info.copy()
        if self.o == 1:
            t = out[3]
            out[3] = out[2]
            out[2] = out[1]
            out[1] = out[0]
            out[0] = t
        if self.o == 2:
            t = out[0]
            out[0] = out[2]
            out[2] = t
            t = out[1]
            out[1] = out[3]
            out[3] = t
        if self.o == 3:
            t = out[1]
            out[1] = out[2]
            out[2] = out[3]
            out[3] = out[0]
            out[0] = t
        return out
            
    def dijkstra(self):
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
                if self.maze[cur][i] == 0:
                    next = self.neighbour(cur[0], cur[1], i)
                    if next in self.maze:
                        if next in weights:
                            if (weights[next] > weights[cur] + self.maze[next][5]):
                                weights[next] = weights[cur] + self.maze[next][5]
                                paths[next] = paths[cur].copy()
                                paths[next].append(next)
                                queue.append(next)
                        else:
                            weights[next] = weights[cur] + self.maze[next][5]
                            paths[next] = paths[cur].copy()
                            paths[next].append(i)
                            queue.append(next)
                    else:
                        dest.append(cur)
        shortest = (0, 0)
        weight = 2000000000
        for i in dest:
            if weights[i] < weight:
                weight = weights[i]
                shortest = i
        self.path = paths[shortest]
class Tile:
    walls = []
    type = 0
    unvisited = 0
    pass

# maze = {}
# o = 0
# x = 0
# y = 0
# count = 0


def go(m, d):
    m.o = (m.o + d) % 4
    if m.o == 0:
        m.y += 1
    if m.o == 1:
        m.x += 1
    if m.o == 2:
        m.y -= 1
    if m.o == 3:
        m.x -= 1
m = Maze()
while True:
    if m.nav() == 1:
        break
    # time.sleep(0.5)
print(m.x, m.y, m.step)
m.printMap()
