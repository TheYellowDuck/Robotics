Maze = [[0b1101, 0b1001, 0b1000, 0b1000, 0b1000, 0b1000, 0b1000, 0b1100],
        [0b0101, 0b0101, 0b0001, 0b0010, 0b0010, 0b0100, 0b0001, 0b0100],
        [0b0101, 0b0111, 0b0101, 0b1001, 0b1000, 0b0100, 0b0001, 0b0100],
        [0b0001, 0b1010, 0b0100, 0b0001, 0b0000, 0b0100, 0b0011, 0b0100],
        [0b0001, 0b1100, 0b0001, 0b0000, 0b0000, 0b0000, 0b1010, 0b0100],
        [0b0001, 0b0100, 0b0001, 0b0000, 0b0010, 0b0100, 0b1111, 0b0101],
        [0b0001, 0b0000, 0b0000, 0b0110, 0b1111, 0b0001, 0b1000, 0b0100],
        [0b0011, 0b0010, 0b0010, 0b1010, 0b1010, 0b0010, 0b0010, 0b0110]]
zx = 3
zy = 2
def Robot(ax, ay):
    input = Maze[zy - ay][zx + ax]
    if o == 1:
        t = (input & 0b1000)
        input[0] = input[1]
        input[1] = input[2]
        input[2] = input[3]
        input[3] = t
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

# 0 = front/up
# 1 = right
# 2 = back/down
# 3 = left

# 0 = blank tile
# -3 = black tile
# -2 = blue tile
# -1 = obstacle
# 1 = ramp
# 2 = checkpoint
# 3 = victim

maze = {}
o = 0
x = 0
y = 0
count = 0

class Tile:
    wall = 0
    unvisited = 0
    type = 0

def direction(d):
    return (d + o) % 4
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
            return (x, y + 1)
        if d == 2:
            return (x + 1, y)
        if d == 3:
            return (x, y - 1)
def printMap():
    print(x, y, count)
    for i in range(7, -8, -1):
        for j in range(-7, 8):
            if (j, i) in maze:
                print(''.join(str (e) for e in maze[(j, i)]), end = ", ")
            else:
                print('****', end = ", ")
        print()
    print()
def nav():
    info = Robot(x, y)
    info.pop(0)
    global count
    walls = 0
    if info[3] == 0 and getCrds(3) not in maze:
        count += 1
        walls += 1
    if info[0] == 0 and getCrds(0) not in maze:
        count += 1
        walls += 1
    if info[1] == 0 and getCrds(1) not in maze:
        count += 1
        walls += 1
    maze[(x, y)] = (save(info), )
    printMap()
    if info[3] == 0 and getCrds(3) not in maze:
        go(3)
        count -= 1
    elif info[0] == 0 and getCrds(0) not in maze:
        go(0)
        count -= 1
    elif info[1] == 0 and getCrds(1) not in maze:
        go(1)
        count -= 1
    else:
        if not count == 0:
        #     go(2)
        # else:
            dijkstra(0, 0)
            return 1
    return 0
def save(info):
    out = info.copy()
    if o == 1:
        t = out[3]
        out[3] = out[2]
        out[2] = out[1]
        out[1] = out[0]
        out[0] = t
    if o == 2:
        t = out[0]
        out[0] = out[2]
        out[2] = t
        t = out[1]
        out[1] = out[3]
        out[3] = t
    if o == 3:
        t = out[1]
        out[1] = out[2]
        out[2] = out[3]
        out[3] = out[0]
        out[0] = t
    return out
        
def dijkstra(a, b):
    pass
def go(d):
    global x
    global y
    global o
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
    o = (o + d) % 4

while True:
    if nav() == 1:
        break