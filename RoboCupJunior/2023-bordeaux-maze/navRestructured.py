import time

class Robot:
    def _init_(self):
        self.rampDir = -1
        
    def check_walls(self):
        pass

class Nav:
    
    def _init_(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.o = 0
        self.path = []
        self.maze = Maze()
        
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
        
        # mark visited
        
        self.info = self.Input()
        
        # update map info
        
        if not self.path:
            self.dfs()
        else:
            self.go(self.maze.getDir(self.o, self.path.pop(0)))
        
        
    def dfs(self):
        if self.info[0] == 0 and self.maze.getCrds(self.o, 0) not in self.maze.visited:
            self.go(self.maze.getDir(self.o, 0))
        elif self.info[3] == 0 and self.maze.getCrds(self.o, 3) not in self.maze.visited:
            self.go(self.maze.getDir(self.o, 3))
        elif self.info[1] == 0 and self.maze.getCrds(self.o, 1) not in self.maze.visited:
            self.go(self.maze.getDir(self.o, 1))
        elif self.info[2] == 0 and self.maze.getCrds(self.o, 2) not in self.maze.visited:
            self.go(self.maze.getDir(self.o, 2))
        else:
            self.optimal_path()
            if self.path:
                self.go(self.maze.getDir(self.o, self.path.pop(0)))
    
    def optimal_path(self):
        queue = [(self.x, self.y, self.z)]
        cost = {(self.x, self.y, self.z) : 0}
        paths = {(self.x, self.y, self.z) : []}
        dest = []
        
        while queue:
            cur = queue.pop(0)
            prevo = self.o
            if len(paths[cur]) != 0:
                prevo = paths[cur][-1]
            for i in range(0, 4):
                if self.maze.maze[cur][i] != 1:
                    next = self.maze.neighbour(cur, i)
                    if next != -1:
                        turnv = 0
                        if prevo != -1:
                            if prevo == (i + 1) % 4 or prevo == (i + 3) % 4:
                                turnv = 0.6
                            if prevo == (i + 2) % 4:
                                turnv = 0.8
                        if next in cost:
                            if (cost[next] > cost[cur] + self.maze.maze[next][4] * (1 + turnv)):
                                cost[next] = cost[cur] + self.maze.maze[next][4] * (1 + turnv)
                                paths[next] = paths[cur].copy()
                                paths[next].append(i)
                                queue.append(next)
                        else:
                            cost[next] = cost[cur] + self.maze.maze[next][4] * (1 + turnv)
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
        # pass
    
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
        robot.go(d)
        
class Maze:
    
    def _init_(self):
        self.maze = {}
        self.visited = []
        self.ramp_count = 0
        self.unvisited_ramp_count = 0
        self.ramps = []
        
    def getDir(self, o, d):
        return (d - o + 4) % 4
    
    def getCrds(self, o, d):
        n = (o + d) % 4
        if n == 0:
            return (navi.x, navi.y + 1, navi.z)
        if n == 1:
            return (navi.x + 1, navi.y, navi.z)
        if n == 2:
            return (navi.x, navi.y - 1, navi.z)
        if n == 3:
            return (navi.x - 1, navi.y, navi.z)
        
    def neighbour(self, tile, d):
        pass
        
navi = Nav()
robot = Robot()
        
        
        